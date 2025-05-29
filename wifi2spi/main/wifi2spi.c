#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/stream_buffer.h"

#include "sdkconfig.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "esp8266/spi_struct.h"
#include "esp8266/gpio_struct.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi.h"

#include "crc.h"

// should SPI packets got from master be CRC checked
#define RXCRC 0

// Interrupt pin, it's switched from low to high
// when ESP8285 has data to send to SPI master
#define INT_GPIO 4

// Data receiving pin, it's is high when ESP8285
// processing received data to indicate that it
// unable to receive new data now
#define GOT_GPIO 0

// UDP send/receive buffer size
#define BUFSIZE	2048

// debug UART buffer size
#define UARTBUFSIZE 512

// Tx/Rx stream buffer size.
#define STREAMBUFSIZE 4096

// wi-fi AP name
#define WIFISSID "copter"

// wi-fi password
#define WIFIPASS "copter12"

// ESP8285 IP
#define SERVIP "192.168.3.1"

// IP assigned to client connected to wi-fi AP
#define CLIENTIP "192.168.3.2"

// UDP port used for wi-fi transmissions
#define PORT 8880

// SPI packet size
#define SPIPACKSIZE 64

// minimum UDP packet size sent to connected client
#define UDPPACKMIN 512

// Rx stream buffer, it is used to store data received from SPI for
// later sending it through wi-fi in corresponding task.
static StreamBufferHandle_t Txbuf;

// Tx stream buffer, it stores data got through wifi that needs
// to be sent through SPI to master.
static StreamBufferHandle_t Rxbuf;

// Flag indicating that send operation is in progress.
static volatile int Sending = 0;

// UDP socket descriptor
volatile int Sock;

// wifi connection event handler. Here it is just a dummy function
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
}

// Send data from Tx stream buffer into SPI
//
// xHigherPriorityTaskWoken -- context switch is needed
static void IRAM_ATTR transfrombuf()
{
	static uint8_t data[SPIPACKSIZE];
	BaseType_t xHigherPriorityTaskWoken;
	static uint16_t cmd;
	uint32_t len;
	spi_trans_t trans;

	// if Tx stream buffer is empty, reset sending flag and return
	if (xStreamBufferBytesAvailable(Txbuf) == 0) {
		Sending = 0;
		return;
	}

	// get data from Tx stream buffer
	len = xStreamBufferReceiveFromISR(Txbuf,
		data + 4, SPIPACKSIZE - 4, &xHigherPriorityTaskWoken);

	// if conext switch is needed, do it
	if (xHigherPriorityTaskWoken == pdTRUE)
		taskYIELD();

	// set packet ID, it is always 0xaa
	data[0] = 0xaa;

	// set packet payload length
	*((uint16_t *) (data + 2)) = len;

	// calculate payload CRC
	data[1] = crc8(data + 2, len + 2);

	// fill transmission structure
	memset(&trans, 0x0, sizeof(trans));
	trans.cmd = &cmd;
	trans.addr = NULL;
	trans.bits.val = 0;
	trans.bits.cmd = 8 * 1;
	trans.bits.addr = 8 * 1;
	trans.bits.mosi = 0;
	trans.miso = (uint32_t *) data;
	trans.bits.miso = SPIPACKSIZE * 8;

	// to prevent context switch between interrupt and
	// start of the transmission, enter critical section
	taskENTER_CRITICAL();

	// switch interrupt pin to high signaling
	// SPI master that new data is coming
	gpio_set_level(INT_GPIO, 1);

	// perform SPI transmission	
	spi_trans(HSPI_HOST, &trans);

	// exit critical section after transmission is done
	taskEXIT_CRITICAL();
}

// SPI event callback.
//
// event -- event type.
// arg -- data sent to callback. In this type of
// 	callback it is a transmission type.
static void IRAM_ATTR spi_event_callback(int event, void* arg)
{
	uint32_t trans_done;
	BaseType_t xHigherPriorityTaskWoken;

	// set got data pin indicating to SPI master that
	// esp8285 unable to receive new data now
	gpio_set_level(GOT_GPIO, 1);

	// if it is not transmission end event, return
	if (event != SPI_TRANS_DONE_EVENT)
		goto skip;

	// retrieve transmission type from callback argument
	trans_done = *(uint32_t*)arg;

	// reset interrupt pin
	gpio_set_level(INT_GPIO, 0);

	// if it is an SPI read transmission done event, send next
	// packet to SPI master
	// if it is an SPI write transmission done event, receive next
	// packet from SPI master
	if (trans_done & SPI_SLV_RD_BUF_DONE)
		transfrombuf();
	else if (trans_done & SPI_SLV_WR_BUF_DONE) {
		uint8_t data[SPIPACKSIZE];
		uint16_t size;
		uint8_t crc;
		uint8_t id;
		int i;

		// retrieve data from SPI read buffer
		for (i = 0; i < 16; ++i)
			((uint32_t *) (data))[i] = SPI1.data_buf[i];

		// get packet ID
		id = data[0];

		// get payload size
		size = *((uint16_t *) (data + 2));
		
		// get payload CRC stored in packet
		crc = data[1];

		// is payload size is greater that maximum possible
		//  or packet ID not 0xaa, discard the packet
		if (size > (SPIPACKSIZE - 4) || id != 0xaa)
			goto skip;

		// if CRC check for incoming packets is enabled
		// and locally calculated CRC doesn't match CRC got
		// from packet, discard the packet
		if (RXCRC && crc8(data + 2, size + 2) != crc)
			goto skip;

		// put packet payload into Rx stream buffer
		xStreamBufferSendFromISR(Rxbuf, (void*) data + 4, size,
			&xHigherPriorityTaskWoken);
	}

	// if conext switch is needed, do it
	if (xHigherPriorityTaskWoken == pdTRUE)
		taskYIELD();

skip:
	// reset got data pin, indication to SPI
	// master that ESP8285 can receive new data
	gpio_set_level(GOT_GPIO, 0);
}

// UDP send task.
//
// pvParameters -- user data sent to task
static void IRAM_ATTR udp_server_task_w(void *pvParameters)
{
	char buf[BUFSIZE];

	// endless loop
	while (1) {
		struct sockaddr_in saddr;
		int len;

		// get data from Rx stream buffer
		len = xStreamBufferReceive(Rxbuf, buf, BUFSIZE,
			portMAX_DELAY);

		// don't send empty packets
		if (len == 0)
			goto skip;

		// set address to send to client's address
		inet_pton(AF_INET, CLIENTIP, &(saddr.sin_addr));
		saddr.sin_family = AF_INET;
		saddr.sin_port = htons(PORT);

		// send data through UDP
		sendto(Sock, buf, len, 0, (struct sockaddr *) &saddr,
			sizeof(saddr));

skip:
		// wait for 5 ms
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

// UDP receive task.
//
// pvParameters -- user data sent to task
static void IRAM_ATTR udp_server_task_r(void *pvParameters)
{
	char buf[BUFSIZE];

	// endless loop
	while (1) {
		int len;
		struct sockaddr_in saddr;
		socklen_t socklen;

		// receive data from UDP
		socklen = sizeof(saddr);
		len = recvfrom(Sock, buf, sizeof(buf), 0,
			(struct sockaddr *) &saddr, &socklen);

		// put received data into Tx stream buffer
		xStreamBufferSend(Txbuf, buf, len, portMAX_DELAY);

		// start sending received data to SPI master
		if (Sending == 0)
			transfrombuf();

		// wait for 10 ms
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

// Initilize GPIO
void gpio_init()
{
	gpio_config_t io_conf;

	// set interrupt and got data pins to output mode
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << INT_GPIO | 1ULL << GOT_GPIO);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	// reset interrupt and got data GPIO pins to low
	gpio_set_level(INT_GPIO, 0);
	gpio_set_level(GOT_GPIO, 0);
}

// Initilize UART
void uart_init()
{
	uart_config_t uart_config;

	uart_config.baud_rate = 921600;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity    = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

	// configure UART
	uart_param_config(UART_NUM_0, &uart_config);

	// install UART driver
	uart_driver_install(UART_NUM_0, UARTBUFSIZE, 0, 0, NULL, 0);
}

// Initilize TCP API
void tcpipapi_init()
{
	tcpip_adapter_ip_info_t ip;
	dhcps_lease_t dhcps_pool;

	// init TCP/IP API
	tcpip_adapter_init();

	// stop DHCP
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));

	// set static IP and network mask to ESP8285 device
	inet_pton(AF_INET, SERVIP, &(ip.ip));
	inet_pton(AF_INET, "255.255.0.0", &(ip.netmask));
	inet_pton(AF_INET, SERVIP, &(ip.gw));

	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP,
		&ip));

	// configure DHCP to give only one
	// and same address to connected client
	dhcps_pool.enable = true;

	inet_pton(AF_INET, CLIENTIP, &(dhcps_pool.start_ip));
	inet_pton(AF_INET, CLIENTIP, &(dhcps_pool.end_ip));

	ESP_ERROR_CHECK(tcpip_adapter_dhcps_option(TCPIP_ADAPTER_OP_SET,
		TCPIP_ADAPTER_REQUESTED_IP_ADDRESS, &dhcps_pool,
		sizeof(dhcps_pool)));

	// start DHCP
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
}

// Initilize SPI
void spiperiph_init()
{
	// set slave mode, polarity 0, phase 0, install callback
	spi_config_t spi_config;
	spi_config.interface.val = SPI_DEFAULT_INTERFACE;
	spi_config.interface.cpol = 0;
	spi_config.interface.cpha = 0;
	spi_config.intr_enable.val = SPI_SLAVE_DEFAULT_INTR_ENABLE;
	spi_config.mode = SPI_SLAVE_MODE;
	spi_config.event_cb = spi_event_callback;
	spi_init(HSPI_HOST, &spi_config);
}

// Initiilze wi-fi
void wifi_init()
{
	wifi_config_t wifi_config;

	// create default wi-fi config
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	// initilize wi-fi hardware
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	// register wi-fi event handler
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
		ESP_EVENT_ANY_ID, wifi_event_handler, NULL));

	// set SSID, password, authorization mode,
	// and channel in created config, reset hidden SSID flag,
	// set allowed maximum connections to 1
	strcpy((char *) &(wifi_config.ap.ssid), WIFISSID);
	wifi_config.ap.ssid_len = strlen(WIFISSID);
	strcpy((char *) &(wifi_config.ap.password), WIFIPASS);
	wifi_config.ap.max_connection = 1;
	wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
	wifi_config.ap.channel = 0;
	wifi_config.ap.ssid_hidden = 0;
	wifi_config.ap.beacon_interval = 100;

	// set AP mode for wi-fi
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

	// install created config
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP,
		&wifi_config));

	// start wi-fi hardware
	ESP_ERROR_CHECK(esp_wifi_start());
}

// Initilize UDP socket
void sock_init()
{
	struct sockaddr_in daddr;

	// create UDP socket
	if ((Sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)) < 0)
		return;

	daddr.sin_addr.s_addr = htonl(INADDR_ANY);
	daddr.sin_family = AF_INET;
	daddr.sin_port = htons(PORT);

	// bind UDP socket to it's address and port
	if (bind(Sock, (struct sockaddr *)&daddr, sizeof(daddr)) < 0)
		return;;
}

// Entry point
void app_main()
{
	// initilize NVS flash
	ESP_ERROR_CHECK(nvs_flash_init());

	// create default ESP event loop
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	// create Tx/Rx stream buffers
	Txbuf = xStreamBufferCreate(STREAMBUFSIZE, 1);
	Rxbuf = xStreamBufferCreate(STREAMBUFSIZE, 1);

	// delay to let SPI master to configure it's pins
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// configure peripherals and TCP/IP API
	gpio_init();
	uart_init();
	tcpipapi_init();
	wifi_init();
	spiperiph_init();

	// initilize UDP socket
	sock_init();

	// start UDP receive task
	xTaskCreate(udp_server_task_r, "udp_server_read",
		8192, NULL, 4, NULL);

	// start UDP send task
	xTaskCreate(udp_server_task_w, "udp_server_write",
		8192, NULL, 4, NULL);
}
