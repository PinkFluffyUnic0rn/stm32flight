#include <string.h>
#include <sys/param.h>

#include "sdkconfig.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/stream_buffer.h"

#include "esp8266/spi_struct.h"


#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/uart.h"
#include "driver/spi.h"
#include "driver/hspi_logic_layer.h"


#define BUF_SIZE 256

#define INT_GPIO 4
#define GOT_GPIO 0

#define RBUF_MAX_SIZE	2048
#define WBUF_MAX_SIZE	2048

#define WIFI_SSID "copter"
#define WIFI_PASS "copter12"

#define SERVIP "192.168.3.1"
#define CLIENTIP "192.168.3.2"
#define PORT 3333

static const uint8_t crsf_crc8tbl [] = {
	0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 
	0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d, 
	0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 
	0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f, 
	0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 
	0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9, 
	0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 
	0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b, 
	0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 
	0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0, 
	0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 
	0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2, 
	0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 
	0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44, 
	0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 
	0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16, 
	0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 
	0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92, 
	0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 
	0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0, 
	0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 
	0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36, 
	0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 
	0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64, 
	0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 
	0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f, 
	0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 
	0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d, 
	0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 
	0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab, 
	0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 
	0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9
};

static EventGroupHandle_t s_connect_event_group;

static StreamBufferHandle_t Txbuf;
static StreamBufferHandle_t Rxbuf;
static int sending_flag = 0;
volatile int Sock;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {
	
	}
}

static uint8_t IRAM_ATTR crc8(const volatile uint8_t *data, uint8_t len)
{
	uint8_t crc;
	int i;

	crc = 0x00;
	for (i = 0; i < len; ++i)
		crc = crsf_crc8tbl[crc ^ *data++];

	return crc;
}

static void IRAM_ATTR transfrombuf(BaseType_t *xHigherPriorityTaskWoken)
{
	static uint8_t data[64];
	static uint16_t cmd;
	uint32_t len;
	spi_trans_t trans;

	if (xStreamBufferBytesAvailable(Txbuf) == 0) {
		sending_flag = 0;
		return;
	}

	len = xStreamBufferReceiveFromISR(Txbuf,
		data + 4, 64 - 4, xHigherPriorityTaskWoken);

	data[0] = 0xaa;
	*((uint16_t *) (data + 2)) = len;
	data[1] = crc8(data + 2, len + 2);

	memset(&trans, 0x0, sizeof(trans));
	trans.cmd = &cmd;
	trans.addr = NULL;
	trans.bits.val = 0;
	trans.bits.cmd = 8 * 1;
	trans.bits.addr = 8 * 1;
	trans.bits.mosi = 0;
	trans.miso = (uint32_t *) data;

	if (len == 0)
		return;

	trans.bits.miso = 64 * 8;
	
	spi_trans(HSPI_HOST, &trans);
	gpio_set_level(INT_GPIO, 1);
}

static void IRAM_ATTR spi_event_callback(int event, void* arg)
{
	uint32_t trans_done;
	BaseType_t xHigherPriorityTaskWoken;
		
	gpio_set_level(GOT_GPIO, 1);

	if (event != SPI_TRANS_DONE_EVENT)
		return;

	trans_done = *(uint32_t*)arg;

	gpio_set_level(INT_GPIO, 0);
	if (trans_done & SPI_SLV_RD_BUF_DONE)
		transfrombuf(&xHigherPriorityTaskWoken);
	else if (trans_done & SPI_SLV_WR_BUF_DONE) {
		uint8_t data[64];
		uint16_t size;
		uint8_t crc;
		uint8_t id;
		int i;
		
		for (i = 0; i < 16; ++i)
			((uint32_t *) (data))[i] = SPI1.data_buf[i];

		id = data[0];
		size = *((uint16_t *) (data + 2));
		crc = data[1];

		if (size > (64 - 4) || crc8(data + 2, size + 2) != crc
				|| id != 0xaa) {
			gpio_set_level(GOT_GPIO, 0);
			return;
		}

		xStreamBufferSendFromISR(Rxbuf, (void*) data + 4, size,
			&xHigherPriorityTaskWoken);
	}
		
	if (xHigherPriorityTaskWoken == pdTRUE)
		taskYIELD();
	
	gpio_set_level(GOT_GPIO, 0);
}

static void IRAM_ATTR udp_server_task_w(void *pvParameters)
{
	char buf[WBUF_MAX_SIZE];
	int ms;

	ms = 0;
	while (1) {
		struct sockaddr_in saddr;
		int len;

		if (xStreamBufferBytesAvailable(Rxbuf) < 512 && ms < 100) {
			ms += 10;
			vTaskDelay(10 / portTICK_PERIOD_MS);
			continue;
		}

		len = xStreamBufferReceive(Rxbuf, buf, WBUF_MAX_SIZE,
			portMAX_DELAY);

		if (len == 0)
			return;

		inet_pton(AF_INET, CLIENTIP, &(saddr.sin_addr));
		saddr.sin_family = AF_INET;
		saddr.sin_port = htons(PORT);
	
		sendto(Sock, buf, len, 0, (struct sockaddr *) &saddr,
			sizeof(saddr));
		
		vTaskDelay(10 / portTICK_PERIOD_MS);

		ms = 0;
	}

	vTaskDelete(NULL);
}

static void IRAM_ATTR udp_server_task_r(void *pvParameters)
{
	char buf[RBUF_MAX_SIZE];

	while (1) {
		int len;
		struct sockaddr_in saddr;
		socklen_t socklen;

		socklen = sizeof(saddr);
		len = recvfrom(Sock, buf, sizeof(buf), 0,
			(struct sockaddr *) &saddr, &socklen);
		
		xStreamBufferSend(Txbuf, buf, len, portMAX_DELAY);

		portENTER_CRITICAL();

		if (sending_flag == 0)
			transfrombuf(NULL);

		portEXIT_CRITICAL();
	
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void gpio_init()
{
	gpio_config_t io_conf;
	
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << INT_GPIO | 1ULL << GOT_GPIO);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);
	gpio_set_level(INT_GPIO, 0);
	gpio_set_level(GOT_GPIO, 0);
}

void uart_init()
{
	uart_config_t uart_config;
	
	uart_config.baud_rate = 921600;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity    = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	
	uart_param_config(UART_NUM_0, &uart_config);
	uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void tcpipapi_init()
{
	tcpip_adapter_ip_info_t ip;
	dhcps_lease_t dhcps_pool;
	
	tcpip_adapter_init();

	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));

	inet_pton(AF_INET, SERVIP, &(ip.ip));
	inet_pton(AF_INET, "255.255.0.0", &(ip.netmask));
	inet_pton(AF_INET, SERVIP, &(ip.gw));

	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP,
		&ip));

	dhcps_pool.enable = true;

	inet_pton(AF_INET, CLIENTIP, &(dhcps_pool.start_ip));
	inet_pton(AF_INET, CLIENTIP, &(dhcps_pool.end_ip));

	ESP_ERROR_CHECK(tcpip_adapter_dhcps_option(TCPIP_ADAPTER_OP_SET,
		TCPIP_ADAPTER_REQUESTED_IP_ADDRESS, &dhcps_pool,
		sizeof(dhcps_pool)));

	ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
}

void spiperiph_init()
{
	spi_config_t spi_config;
	spi_config.interface.val = SPI_DEFAULT_INTERFACE;
	spi_config.interface.cpol = 0;
	spi_config.interface.cpha = 0;
	spi_config.intr_enable.val = SPI_SLAVE_DEFAULT_INTR_ENABLE;
	spi_config.mode = SPI_SLAVE_MODE;
	spi_config.event_cb = spi_event_callback;
	spi_init(HSPI_HOST, &spi_config);
}

void wifi_init()
{
	wifi_config_t wifi_config;
	
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
		ESP_EVENT_ANY_ID, wifi_event_handler, NULL));

	strcpy((char *) &(wifi_config.ap.ssid), WIFI_SSID);
	wifi_config.ap.ssid_len = strlen(WIFI_SSID);
	strcpy((char *) &(wifi_config.ap.password), WIFI_PASS);
	wifi_config.ap.max_connection = 1;
	wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
	wifi_config.ap.channel = 0;
	wifi_config.ap.ssid_hidden = 0;
	wifi_config.ap.beacon_interval = 100;

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP,
		&wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

void sock_init()
{
	char addr_str[128];
	struct sockaddr_in daddr;

	daddr.sin_addr.s_addr = htonl(INADDR_ANY);
	daddr.sin_family = AF_INET;
	daddr.sin_port = htons(PORT);

	inet_ntoa_r(daddr.sin_addr, addr_str, sizeof(addr_str) - 1);

	if ((Sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)) < 0)
		return;

	if (bind(Sock, (struct sockaddr *)&daddr, sizeof(daddr)) < 0)
		return;;
}

void app_main()
{

	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	Txbuf = xStreamBufferCreate(4096, 1);
	Rxbuf = xStreamBufferCreate(4096, 1);
	
	if (s_connect_event_group != NULL)
		return;

	s_connect_event_group = xEventGroupCreate();

	vTaskDelay(1000 / portTICK_PERIOD_MS);

	gpio_init();

	uart_init();

	tcpipapi_init();
	
	wifi_init();

	spiperiph_init();

	sock_init();
    
	ESP_LOGI("LOG", "HELLO!\r\n");

	xTaskCreate(udp_server_task_r, "udp_server_read",
		8192, NULL, 4, NULL);

	xTaskCreate(udp_server_task_w, "udp_server_write",
		8192, NULL, 4, NULL);
}
