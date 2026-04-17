/**
* @file m10.h
* @brief M10 GNSS device driver
*/

#ifndef M10_H
#define M10_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define M10_MAXDEVS 1

/**
* @brief does GNSS has fix
* @param qual quality value
*/
#define M10_HASFIX(qual) ((qual) == M10_QUAL_SINGLE		\
	|| (qual) == M10_QUAL_PSEUDORANGE			\
	|| (qual) == M10_QUAL_FIXEDAMB				\
	|| (qual) == M10_QUAL_FLOATAMB				\
	|| (qual) == M10_QUAL_PSEUDORANGE)

/**
* @brief device initialization and private data
*/
struct m10_device {
	UART_HandleTypeDef *huart;	/*!< UART interface */
};

/**
* GNSS connection quality
*/
enum M10_QUALITY
{
	M10_QUAL_NOFIX = 0,		/*!< no fix */
	M10_QUAL_SINGLE = 1,		/*!< fix */
	M10_QUAL_PSEUDORANGE = 2,	/*!< differention fix */
	M10_QUAL_FIXEDAMB = 4,		/*!< RTK fix */
	M10_QUAL_FLOATAMB = 5,		/*!< RTK float */
	M10_QUAL_DEADRECK = 6,		/*!< dead reckoning */
	M10_QUAL_MANUALINP = 7,		/*!< manual input */
	M10_QUAL_SIMULATOR = 8,		/*!< simulation */
	M10_QUAL_WAAS = 9		/*!< WAAS fix */
};

/**
* NMEA message type
*/
enum M10_TYPE
{
	M10_TYPE_GGA = 0,	/*!< GGA message */
	M10_TYPE_RMC = 1,	/*!< RMC message */
	M10_TYPE_OTHER = 2,	/*!< other message */
};

/**
* M10 NMEA output message
*/
struct m10_data {
	enum M10_TYPE type;			/*!< message type */
	char msg[83];				/*!< raw message */

	/**
	* NMEA message
	*/
	union {						
		/**
		* GGA message
		*/
		struct {
			float time;			/*!< current time */
			uint8_t lat;			/*!< latitude degrees */
			float latmin;			/*!< latitude minutes */
			char latdir;			/*!< latitude hemisphere */
			uint8_t lon;			/*!< longitude degrees */
			char londir;			/*!< longitude hemisphere */
			float lonmin;			/*!< longitude minute */
			enum M10_QUALITY quality;	/*!< connection quality */
			uint8_t sats;			/*!< connected satellites */
			float alt;			/*!< altitude */
		} gga;
		
		/**
		* RMC message
		*/
		struct {
			float time;		/*!< current time */
			uint8_t fstatus;	/*!< data status */
			uint8_t lat;		/*!< latitude degrees */
			float latmin;		/*!< latitude minutes */
			char latdir;		/*!< latitude hemisphere */
			uint8_t lon;		/*!< longitude degrees */
			char londir;		/*!< longitude hemispere */
			float lonmin;		/*!< longitude minutes */
			float speed;		/*!< speed */
			float course;		/*!< course */
			char date[10];		/*!< date */
			float magvar;		/*!< magnetic variation */
			uint8_t magvardir;	/*!< magnetic variation
							direction */
		} rmc;
	};
};

/**
* @brief initialize M10 device.
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int m10_initdevice(struct m10_device *is, struct cdevice *dev);

#endif
