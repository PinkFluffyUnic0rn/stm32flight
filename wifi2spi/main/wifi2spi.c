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

#include "crc.h"

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
