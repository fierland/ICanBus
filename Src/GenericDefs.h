#ifndef __GENERICDEFS_H_
#define __GENERICDEFS_H_

// only for debug and code writing is defined in project settings
#ifndef ICAN_MASTER
#define ICAN_MASTER2
#endif
#ifndef ICAN_INSTRUMENT
#define ICAN_INSTRUMENT2
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include <esp_task_wdt.h>

extern QueueHandle_t xQueueRREF;	// queu for results of data items from Xplane
extern QueueHandle_t xQueueDREF;	// queue for data items to send to XPlane
extern QueueHandle_t xQueueDataSet;  // queue to  request of stop dataitems in XPlane interface
//extern QueueHandle_t xQueueStatus;	// queue for status messages

// event groups
/* FreeRTOS event group to signal when we are connected*/
extern EventGroupHandle_t s_connection_event_group;

const int RUNNING_CANBUS_BIT = BIT0;
const int POWER_ON_BIT = BIT1;
const int PLANE_CONNECTED_BIT = BIT2;

#ifdef ICAN_MASTER
const int UDP_CONNECTED_BIT = BIT3;
const int BEACON_CONNECTED_BIT = BIT4;
const int CLEANING_CONNECTED_BIT = BIT5;
const int WIFI_CONNECTED_BIT = BIT6;
const int RUNNING_CONNECTED_BIT = BIT7;
#endif

#define _instrumentIsPowered (xEventGroupGetBits(s_connection_event_group) & POWER_ON_BIT)
#define _canbusIsRunning (xEventGroupGetBits(s_connection_event_group) & RUNNING_CANBUS_BIT)
#define _planeIsConnected (xEventGroupGetBits(s_connection_event_group) & PLANE_CONNECTED_BIT)

typedef enum {
	CANAS_DATATYPE_NODATA,
	CANAS_DATATYPE_ERROR,

	CANAS_DATATYPE_FLOAT,

	CANAS_DATATYPE_LONG,
	CANAS_DATATYPE_ULONG,
	CANAS_DATATYPE_BLONG,

	CANAS_DATATYPE_SHORT,
	CANAS_DATATYPE_USHORT,
	CANAS_DATATYPE_BSHORT,

	CANAS_DATATYPE_CHAR,
	CANAS_DATATYPE_UCHAR,
	CANAS_DATATYPE_BCHAR,

	CANAS_DATATYPE_SHORT2,
	CANAS_DATATYPE_USHORT2,
	CANAS_DATATYPE_BSHORT2,

	CANAS_DATATYPE_CHAR4,
	CANAS_DATATYPE_UCHAR4,
	CANAS_DATATYPE_BCHAR4,

	CANAS_DATATYPE_CHAR2,
	CANAS_DATATYPE_UCHAR2,
	CANAS_DATATYPE_BCHAR2,

	CANAS_DATATYPE_MEMID,
	CANAS_DATATYPE_CHKSUM,

	CANAS_DATATYPE_ACHAR,
	CANAS_DATATYPE_ACHAR2,
	CANAS_DATATYPE_ACHAR4,

	CANAS_DATATYPE_CHAR3,
	CANAS_DATATYPE_UCHAR3,
	CANAS_DATATYPE_BCHAR3,
	CANAS_DATATYPE_ACHAR3,

	CANAS_DATATYPE_DOUBLEH,
	CANAS_DATATYPE_DOUBLEL,

	CANAS_DATATYPE_RESVD_BEGIN_,
	CANAS_DATATYPE_RESVD_END_ = 99,

	CANAS_DATATYPE_UDEF_BEGIN_ = 100,
	CANAS_DATATYPE_STRING,
	CANAS_DATATYPE_UDEF_END_ = 255,

	CANAS_DATATYPE_ALL_END_ = 255
} CanasStandardDataTypeID;

typedef union {
	uint32_t ERROR;

	float FLOAT;

	int32_t  LONG;
	uint32_t ULONG;
	uint32_t BLONG;

	int16_t  SHORT;
	uint16_t USHORT;
	uint16_t BSHORT;

	int8_t  CHAR;
	uint8_t UCHAR;
	uint8_t BCHAR;

	int16_t  SHORT2[2];
	uint16_t USHORT2[2];
	uint16_t BSHORT2[2];

	int8_t  CHAR4[4];
	uint8_t UCHAR4[4];
	uint8_t BCHAR4[4];

	int8_t  CHAR2[2];
	uint8_t UCHAR2[2];
	uint8_t BCHAR2[2];

	uint32_t MEMID;
	uint32_t CHKSUM;

	uint8_t ACHAR;
	uint8_t ACHAR2[2];
	uint8_t ACHAR4[4];

	int8_t  CHAR3[3];
	uint8_t UCHAR3[3];
	uint8_t BCHAR3[3];
	uint8_t ACHAR3[3];

	uint32_t DOUBLEH;
	uint32_t DOUBLEL;
} CanasDataContainer;

typedef struct {
	uint8_t type = CANAS_DATATYPE_FLOAT;    ///< @ref InsCanStandardDataTypeID or custom
	uint8_t length = 4;							///< Ignored with standard datatypes; required otherwise. Leave zero if unused.
	CanasDataContainer container;
} CanasMessageData;

#ifdef ICAN_INSTRUMENT
const int MASTER_RUNNING_BIT = BIT3;
#endif

typedef struct queueDataSetItem {
	uint16_t	canId;
	int			interval = 1;
	bool		send = false;
};

typedef struct queueDataItem {
	uint16_t			canId;
	CanasMessageData	data;
};

#endif
