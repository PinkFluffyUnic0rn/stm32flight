#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "util.h"
#include "irc.h"

#define IRC_TIMEOUT 1000
#define IRC_RETRIES 5

static struct irc_device irc_devs[IRC_MAXDEVS];
static size_t irc_devcount = 0;

uint8_t irc_checksum(uint8_t *data, size_t sz)
{
	uint8_t sum;
	int i;

	sum = 0;
	for (i = 0; i < sz; ++i)
		sum += data[i];

	return sum;
}

int irc_sendpacket(struct irc_device *dev, uint8_t cmd, int par)
{
	uint8_t data[16];
	
	memset(data, 0, 16);

	data[0] = 0x0f;
	data[1] = cmd;
	data[2] = par & 0xff;
	data[3] = (par >> 8) & 0xff;
	data[14] = irc_checksum(data + 1, 13);
	data[15] = 0x0;

	HAL_HalfDuplex_EnableTransmitter(dev->huart);
	HAL_UART_Transmit(dev->huart, data, 16, 1000);
	HAL_HalfDuplex_EnableReceiver(dev->huart);

	return 0;
}

int irc_receivepacket(struct irc_device *dev, uint8_t *data)
{
	HAL_UART_Receive(dev->huart, data, 16, 1000);

	return 0;
}

int irc_read(void *d, void *dt, size_t sz)
{
	struct irc_data *data;
	struct irc_device *dev;
	uint8_t pack[16];
	int retries;

	data = dt;
	dev = d;

	for (retries = 0; retries < IRC_RETRIES; ++retries) {
		irc_sendpacket(dev, 'v', dev->power);

		HAL_UART_Receive(dev->huart, pack, 16, 1000);

		if (irc_checksum(pack + 1, 13) == pack[14])
			break;
	}

	data->frequency = pack[2] | pack[3] << 8;
	data->power = pack[4] | pack[5] << 8;

	return 0;
}

int irc_configure(void *d, const char *cmd, ...)
{
	struct irc_device *dev;
	struct irc_data data;
	int retries;
	va_list args;
	
	dev = d;
	
	va_start(args, cmd);

	if (strcmp(cmd, "frequency") == 0) {
		int freq;

		freq = va_arg(args, int);

		for (retries = 0; retries < IRC_RETRIES; ++retries) {
			irc_sendpacket(dev, 'F', freq);
			HAL_Delay(250);
			
			if (irc_read(d, &data,
					sizeof(struct irc_data)) != 0)
				continue;

			if (data.frequency == freq)
				break;
		}

		if (retries == IRC_RETRIES)
			return (-1);	

		dev->frequency = freq;
	}
	else if (strcmp(cmd, "power") == 0) {
		int power;
		
		power = va_arg(args, int);
	
		for (retries = 0; retries < IRC_RETRIES; ++retries) {	
			irc_sendpacket(dev, 'P', power);
			HAL_Delay(250);
	
			if (irc_read(d, &data,
					sizeof(struct irc_data)) != 0)
				continue;

			if (data.power == power)
				break;
		}	

		if (retries == IRC_RETRIES)
			return (-1);	

		dev->power = power;
	}
	
	va_end(args);

	return 0;
}

int irc_init(struct irc_device *dev)
{
	uint8_t data[16];
	int retries;

	for (retries = 0; retries < IRC_RETRIES; ++retries) {
		irc_sendpacket(dev, 'r', 0);

		HAL_UART_Receive(dev->huart, data, 16, 1000);

		if (irc_checksum(data + 1, 13) == data[14])
			break;
	}

	if (retries == IRC_RETRIES)
		return (-1);

	if (irc_configure(dev, "power", dev->power) < 0)
		return (-1);

	if (irc_configure(dev, "frequency", dev->frequency) < 0)
		return (-1);

	return 0;
}

int irc_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(irc_devs + irc_devcount, is, sizeof(struct irc_device));

	sprintf(dev->name, "%s_%d", "irc", irc_devcount);

	dev->priv = irc_devs + irc_devcount;
	dev->read = irc_read;
	dev->configure = irc_configure;
	dev->write = NULL;
	dev->interrupt = NULL;
	dev->error = NULL;

	dev->status = DEVSTATUS_INIT;

	r = irc_init(irc_devs + irc_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
