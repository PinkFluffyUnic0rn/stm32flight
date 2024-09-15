#ifndef CRSF_H
#define CRSF_H

#define CRSF_CHANNELCOUNT 10

struct crsf_device {
	UART_HandleTypeDef *huart;
};

struct crsf_channels {
	float chf[CRSF_CHANNELCOUNT];
};


int crsf_interrupt(struct crsf_device *dev, const void *h);

int crsf_read(struct crsf_channels *c);

int crsf_init(struct crsf_device *dev);

#endif
