#include "stm32f3xx_hal.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "uartdebug.h"
#include "esp8266.h"

#define ESP_BAUDRATE 921600
#define ESP_UARTTIMEOUT 100
#define ESP_TIMEOUT 1000
#define ESP_JOINTIMEOUT 30000
#define ESP_FIFOSIZE 8

enum ESP_RESPONSE {
	ESP_OK = 0,
	ESP_FAIL = 1,
	ESP_ERROR = 2,
	ESP_ALREADYCONNECTED = 3,
	ESP_UNKNOWN = 4,
};

struct fifo {
	char cmd[ESP_FIFOSIZE][ESP_CMDSIZE];
	size_t bot;
	size_t top;
};

static uint8_t Rxbyte;

static volatile struct fifo fifo;

void memcpyv(volatile void *dest, const volatile void *src, size_t n)
{
	int i;

	for (i = 0; i < n; ++i)
		((uint8_t *) dest)[i] = ((uint8_t *) src)[i];
}

int esp_initfifo(volatile struct fifo *f)
{
	f->bot = f->top = 0;

	return 0;
}

int esp_dequeque(volatile struct fifo *f, char *out)
{
	if (f->bot == f->top)
		return (-1);
	
	memcpyv(out, f->cmd[f->bot], ESP_CMDSIZE);

	f->bot = (f->bot + 1) % ESP_FIFOSIZE;

	return 0;
}

int esp_waitforstrings(volatile struct fifo *f, int timeout,
	char *res, ...)
{
	int t;

	for (t = 0; t < timeout; t += 1) {
		va_list args;
		char *s;
		int i;
		
		if (esp_dequeque(f, res) < 0) {
			HAL_Delay(1);
			continue;
		}

		va_start(args, res);

		i = 0;
		while ((s = va_arg(args, char *)) != NULL) {
			if (strncmp(res, s, strlen(s)) == 0) {
				va_end(args);
				return i;
			}

			++i;
		}

		va_end(args);

		HAL_Delay(1);
	}

	return (-1);
}

int esp_cmd(struct esp_device *dev,
	char *res, int timeout, const char *cmd)
{
	char buf[ESP_CMDSIZE];
	int r;

	sprintf(buf, "%s\r\n", cmd);

	HAL_UART_Transmit(dev->huart, (uint8_t *) buf,
		strlen(buf), ESP_UARTTIMEOUT);

	r = esp_waitforstrings(&fifo, timeout, buf, "OK", "FAIL",
		"ERROR", "ALREADY CONNECTED", NULL);

	if (res != NULL)
		memcpy(res, buf, ESP_CMDSIZE);

	return r;
}

int esp_getip(struct esp_device *dev, char *ipout)
{
	char res[ESP_CMDSIZE];
	char *p, *ip;

	HAL_UART_Transmit(dev->huart, (uint8_t *) "AT+CIFSR\r\n",
		strlen("AT+CIFSR\r\n"), ESP_UARTTIMEOUT);

	if (esp_waitforstrings(&fifo, ESP_TIMEOUT, res,
			"+CIFSR:STAIP,", NULL) < 0)
		return (-1);

	ip = res + strlen("+CIFSR:STAIP,\"");
	for (p = ip; *p != '\"' && *p != '\0'; ++p);

	*p = '\0';

	strcpy(ipout, ip);

	if (esp_waitforstrings(&fifo, ESP_TIMEOUT, res, "OK", NULL) < 0)
		return (-1);

	return 0;
}

int esp_init(struct esp_device *dev, const char *ssid, const char *pass,
	int port)
{
	char cmd[ESP_CMDSIZE];

	esp_initfifo(&fifo);
	
	dev->status = ESP_FIFOINIT;

	HAL_UART_Receive_DMA(dev->huart, &Rxbyte, 1);

	sprintf(cmd, "AT+UART_CUR=%d,8,1,0,0", ESP_BAUDRATE);
	esp_cmd(dev, NULL, ESP_TIMEOUT, cmd);

	dev->status = ESP_NOINIT;

	HAL_UART_DeInit(dev->huart);

	dev->huart->Init.BaudRate = ESP_BAUDRATE;

	if (HAL_UART_Init(dev->huart) != HAL_OK)
		return (-1);	

	esp_initfifo(&fifo);
	
	dev->status = ESP_FIFOINIT;

	HAL_UART_Receive_DMA(dev->huart, &Rxbyte, 1);
	
	if (esp_cmd(dev, NULL, ESP_TIMEOUT, "ATE0") != ESP_OK)
		return (-1);

	if (esp_cmd(dev, NULL, ESP_TIMEOUT, "AT") != ESP_OK)
		return (-1);

	if (esp_cmd(dev, NULL, ESP_TIMEOUT, "AT+CWMODE=2") != ESP_OK)
		return (-1);

	if (esp_cmd(dev, NULL, ESP_TIMEOUT, "AT+CWDHCP=0,1") != ESP_OK)
		return (-1);

	sprintf(cmd, "AT+CWSAP_CUR=\"%s\",\"%s\",5,0", ssid, pass);
	if (esp_cmd(dev, NULL, ESP_TIMEOUT, cmd) != ESP_OK)
		return (-1);
		
	if (esp_cmd(dev, NULL, ESP_TIMEOUT,
		"AT+CIPAP_CUR=\"192.168.3.1\",\"192.168.3.1\",\"255.255.255.0\"") != ESP_OK)
		return (-1);
		
	if (esp_cmd(dev, NULL, ESP_TIMEOUT,
		"AT+CWDHCPS_CUR=1,2880,\"192.168.3.2\",\"192.168.3.2\"") != ESP_OK)
		return (-1);

	dev->status = ESP_INIT;

	esp_cmd(dev, NULL, ESP_TIMEOUT, "AT+CIPCLOSE");

	sprintf(cmd, "AT+CIPSTART=\"UDP\",\"192.168.3.2\",%d,%d,0",
		port, port);

	if (esp_cmd(dev, NULL, ESP_TIMEOUT, cmd) != ESP_OK)
		return (-1);

	dev->status = ESP_CONNECTED;

	return 0;
}

int esp_connect(struct esp_device *dev, const char *ip, int port)
{
	char s[ESP_CMDSIZE];
	int i;

	sprintf(s, "AT+CIPSTART=\"UDP\",\"%s\",%d,%d,0",
		ip, port, port);

	for (i = 0; i < ESP_CONNRETRIES; ++i) {
		if (esp_cmd(dev, NULL, ESP_TIMEOUT, s) == ESP_OK)
			break;

		esp_disconnect(dev);
	}

	if (i >= ESP_CONNRETRIES)
		return (-1);

	return 0;
}

int esp_disconnect(struct esp_device *dev)
{
	if (esp_cmd(dev, NULL, ESP_TIMEOUT, "AT+CIPCLOSE") != ESP_OK)
		return (-1);

	return 0;
}

int esp_interrupt(struct esp_device *dev, const void *h)
{
	static size_t Rxoffset = 0;

	if (((UART_HandleTypeDef *)h)->Instance != dev->huart->Instance)
		return 0;

	fifo.cmd[fifo.top][Rxoffset] = Rxbyte;

	if (fifo.cmd[fifo.top][Rxoffset] == '\n') {
		fifo.cmd[fifo.top][Rxoffset] = '\0';
	
		if ((fifo.top + 1) % ESP_FIFOSIZE != fifo.bot) {
			fifo.cmd[fifo.top][Rxoffset] = '\0';

			fifo.top = (fifo.top + 1) % ESP_FIFOSIZE;
		}

		Rxoffset = 0;
	}
	else if (Rxoffset++ == (ESP_CMDSIZE - 1))
		Rxoffset = 0;

	return 0;
}

int esp_error(struct esp_device *dev, const void *h)
{
	if (((UART_HandleTypeDef *)h)->Instance != dev->huart->Instance)
		return 0;

	if (dev->huart->ErrorCode) {
		__HAL_UART_CLEAR_OREFLAG(dev->huart);
		__HAL_UART_CLEAR_NEFLAG(dev->huart);
		__HAL_UART_CLEAR_FEFLAG(dev->huart);

		HAL_UART_DeInit(dev->huart);

		HAL_UART_Init(dev->huart);

		HAL_UART_Receive_DMA(dev->huart, &Rxbyte, 1);
	}

	return 0;
}

static enum ESP_RESPONSE _esp_send(struct esp_device *dev, int timeout,
	const char *data)
{
	char buf[ESP_CMDSIZE];
	size_t rem, off;
	int r;

	rem = strlen(data);
	off = 0;

	sprintf(buf, "AT+CIPSEND=%d\r\n", rem);

	HAL_UART_Transmit(dev->huart, (uint8_t *) buf,
		strlen(buf), ESP_UARTTIMEOUT);

	esp_waitforstrings(&fifo, timeout, buf, "OK", NULL);

	while (rem > 0) {
		size_t len;

		len = (rem < (ESP_CMDSIZE - 1) ? rem : (ESP_CMDSIZE - 1));

		HAL_UART_Transmit(dev->huart, (uint8_t *) (data + off),
			len, ESP_UARTTIMEOUT);
	
		rem -= len;
		off += len;
	}

	r = esp_waitforstrings(&fifo, timeout, buf,
		"SEND OK", "ERROR", "SEND FAIL", NULL);

	switch (r) {
	case 0: return ESP_OK;
	case 1: return ESP_ERROR;
	case 2: return ESP_FAIL;
	}

	return ESP_UNKNOWN;
}

int esp_send(struct esp_device *dev, const char *data)
{
	if (_esp_send(dev, ESP_TIMEOUT, data) != ESP_OK)
		return (-1);

	return 0;
}

int esp_poll(struct esp_device *dev, char *outdata)
{
	char res[ESP_CMDSIZE];
	char *ssz, *data;

	if (esp_dequeque(&fifo, res) < 0)
		return (-1);

	if ((ssz = strstr((char *) res, "+IPD,")) == NULL)
		return (-1);
	
	if ((ssz = strchr(ssz, ',')) == NULL)
		return (-1);

	ssz++;

	if ((data = strchr(ssz, ':')) == NULL)
		return (-1);

	*data++ = '\0';

	if (strlen(data) >= ESP_CMDSIZE)
		return (-1);

	memcpy(outdata, data, strlen(data) + 1);

	return 0;
}
