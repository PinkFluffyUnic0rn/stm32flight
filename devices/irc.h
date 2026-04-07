/**
* @file irc.h
* @brief IRC VTX device driver
*/

#ifndef IRC_H
#define IRC_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define IRC_MAXDEVS 1

/**
* @brief VTX power table (of specific one)
*/
enum IRC_POWER {
	IRC_POWER_25	= 25,	/*!< 25 mW */
	IRC_POWER_100	= 100,	/*!< 100 mw */
	IRC_POWER_200	= 200,	/*!< 200 mW */
	IRC_POWER_400	= 400,	/*!< 400 mW */
	IRC_POWER_600	= 600	/*!< 600 mW */
};

/**
* @brief VTX frequency table (of specific one), values in mHz
*/
enum IRC_FREQ {
	IRC_FREQ_5865 = 5865,	IRC_FREQ_5845 = 5845,
	IRC_FREQ_5825 = 5825,	IRC_FREQ_5805 = 5805,
	IRC_FREQ_5785 = 5785,	IRC_FREQ_5765 = 5765,
	IRC_FREQ_5745 = 5745,	IRC_FREQ_5725 = 5725,

	IRC_FREQ_5733 = 5733,	IRC_FREQ_5752 = 5752,
	IRC_FREQ_5771 = 5771,	IRC_FREQ_5790 = 5790,
	IRC_FREQ_5809 = 5809,	IRC_FREQ_5828 = 5828,
	IRC_FREQ_5847 = 5847,	IRC_FREQ_5866 = 5866,

	IRC_FREQ_5705 = 5705,	IRC_FREQ_5685 = 5685,
	IRC_FREQ_5665 = 5665,	IRC_FREQ_5645 = 5645,
	IRC_FREQ_5885 = 5885,	IRC_FREQ_5905 = 5905,
	IRC_FREQ_5925 = 5925,	IRC_FREQ_5945 = 5945,

	IRC_FREQ_5740 = 5740,	IRC_FREQ_5760 = 5760,
	IRC_FREQ_5780 = 5780,	IRC_FREQ_5800 = 5800,
	IRC_FREQ_5820 = 5820,	IRC_FREQ_5840 = 5840,
	IRC_FREQ_5860 = 5860,	IRC_FREQ_5880 = 5880,

	IRC_FREQ_5658 = 5658,	IRC_FREQ_5695 = 5695,
	IRC_FREQ_5732 = 5732,	IRC_FREQ_5769 = 5769,
	IRC_FREQ_5806 = 5806,	IRC_FREQ_5843 = 5843,
				IRC_FREQ_5917 = 5917
};

/**
* @brief IRC VTX data
*/
struct irc_data {
	int power;	/*!< VTX power */
	int freq;	/*!< VTX frequency */
};

/**
* @brief device initialization and private data
*/
struct irc_device {
	UART_HandleTypeDef *huart;	/*!< UART interface */
	enum IRC_POWER power;		/*!< VTX power */
	enum IRC_FREQ freq;		/*!< VTX frequency */
};

/**
* @brief power validation against power table.
*
* @param dev power value
* @return 1 is power value is valid, 0 otherwise
*/
int irc_ispowervalid(int v);

/**
* @brief frequency validation against power table.
*
* @param dev frequency value
* @return 1 is frequency value is valid, 0 otherwise
*/
int irc_isfreqvalid(int v);

/**
* @brief initialize IRC VTX device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
*/
int irc_initdevice(void *is, struct cdevice *dev);

#endif
