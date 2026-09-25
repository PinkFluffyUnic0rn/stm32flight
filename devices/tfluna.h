/**
* @file crsf.h
* @brief TF-Luna device driver
*/

#ifndef TFLUNA_H
#define TFLUNA_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define TF_MAXDEVS 1

/**
* @brief device initialization and private data
*/
struct tf_device {
	UART_HandleTypeDef *huart;	/*!< UART interface */
};

/**
* @brief output data
*/
struct tf_data {
	double dist;
	double amp;
	double temp;
};

/**
* @brief initialize TF-Luna device.
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int tf_initdevice(struct tf_device *is, struct cdevice *dev);

#endif
