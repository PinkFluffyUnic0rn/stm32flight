#ifndef ESP_INIT_H
#define ESP_INIT_H

#define ESP_CONNRETRIES 10
#define ESP_CMDSIZE 128

enum ESP_DEVSTATUS {
	ESP_CONNECTED	= 0,
	ESP_INIT	= 1,
	ESP_FIFOINIT	= 2,
	ESP_FAILED	= 3,
	ESP_NOINIT	= 4
};

struct esp_device {
	UART_HandleTypeDef *huart;
	volatile int status;
};

int esp_init(struct esp_device *dev, const char *ssid,
	const char *pass, int port);

int esp_getip(struct esp_device *dev, char *ip);

int esp_connect(struct esp_device *dev, const char *ip, int port);

int esp_disconnect(struct esp_device *dev);

int esp_interrupt(struct esp_device *dev, const void *huart);

int esp_error(struct esp_device *dev, const void *huart);

int esp_send(struct esp_device *dev, const char *data);

int esp_printf(struct esp_device *dev, const char *format, ...);

int esp_poll(struct esp_device *dev, char *outdata);

#endif
