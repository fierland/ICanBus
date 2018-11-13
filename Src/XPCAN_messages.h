// ================================================================================================
// Instrument_Can interface : Message definitions
//
// Based on CanAreospace and the code from Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
// ================================================================================================
//#pragma once

#ifndef INSTRUMENTCAN_MESSAGE_H_
#define INSTRUMENTCAN_MESSAGE_H_

#include <stdint.h>

// global variables and limits around the ICanbus implementation

constexpr uint8_t ICAN_MIN_SOFTWARE_REVISION = 0;
constexpr uint8_t CANAS_BROADCAST_NODE_ID = 0;
constexpr uint8_t XI_Base_NodeID = 1;
constexpr auto ICAN_BAUD_RATE = 500000;

static const int CANAS_DEFAULT_REPEAT_TIMEOUT_USEC = 30 * 1000;
/**
 * At least 10ms is required by some services.
 * Thus, increasing is NOT recommended.
 */
static const int CANAS_DEFAULT_SERVICE_POLL_INTERVAL_USEC = 10 * 1000;

/**
 * History length for repetition detection.
 * Expressed in number of frames to track.
 */
static const int CANAS_DEFAULT_SERVICE_HIST_LEN = 32;
/**
 * Time to wait for response from remote node
 */
static const int CANAS_DEFAULT_SERVICE_REQUEST_TIMEOUT_USEC = 100 * 1000;

static const int  CANAS_DEFAULT_SERVICE_ADVERTISE_TIMEOUT_USEC = 20 * 1000 * 1000;

typedef enum {
	CANAS_MSGTYPE_EMERGENCY_EVENT_MIN = 0,
	CANAS_MSGTYPE_EMERGENCY_EVENT_MAX = 127,

	CANAS_MSGTYPE_NODE_SERVICE_HIGH_MIN = 128,
	CANAS_MSGTYPE_NODE_SERVICE_HIGH_MAX = 199,

	CANAS_MSGTYPE_USER_DEFINED_HIGH_MIN = 200,
	CANAS_MSGTYPE_USER_DEFINED_HIGH_MAX = 299,

	CANAS_MSGTYPE_NORMAL_OPERATION_MIN = 300,
	CANAS_MSGTYPE_NORMAL_OPERATION_MAX = 1799,

	CANAS_MSGTYPE_USER_DEFINED_LOW_MIN = 1800,
	CANAS_MSGTYPE_USER_DEFINED_LOW_MAX = 1899,

	CANAS_MSGTYPE_DEBUG_SERVICE_MIN = 1900,
	CANAS_MSGTYPE_DEBUG_SERVICE_MAX = 1999,

	CANAS_MSGTYPE_NODE_SERVICE_LOW_MIN = 2000,
	CANAS_MSGTYPE_NODE_SERVICE_LOW_MAX = 2031
} CanasMessageTypeID;

typedef enum {
	CANAS_SERVICE_CHANNEL_HIGH_MIN = 0,
	CANAS_SERVICE_CHANNEL_HIGH_MAX = 35,

	CANAS_SERVICE_CHANNEL_LOW_MIN = 100,
	CANAS_SERVICE_CHANNEL_LOW_MAX = 115
} CanasServiceChannelID;

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
	uint8_t type;                 ///< @ref InsCanStandardDataTypeID or custom
	uint8_t length;               ///< Ignored with standard datatypes; required otherwise. Leave zero if unused.
	CanasDataContainer container;
} CanasMessageData;

typedef struct {
	CanasMessageData data;
	uint8_t 	node_id;
	uint8_t 	service_code;
	uint8_t 	message_code;
	uint16_t 	can_id;
} CanasMessage;

typedef enum {
	MSGGROUP_WTF,
	MSGGROUP_PARAMETER,
	MSGGROUP_SERVICE
} MessageGroup;

typedef enum {
	ICAN_ERR_OK = 0,

	ICAN_ERR_ARGUMENT,
	ICAN_ERR_NOT_ENOUGH_MEMORY,

	ICAN_ERR_DRIVER,

	ICAN_ERR_NO_SUCH_ENTRY,
	ICAN_ERR_ENTRY_EXISTS,

	ICAN_ERR_BAD_DATA_TYPE,
	ICAN_ERR_BAD_MESSAGE_ID,
	ICAN_ERR_BAD_NODE_ID,
	ICAN_ERR_BAD_REDUND_CHAN,
	ICAN_ERR_BAD_SERVICE_CHAN,
	ICAN_ERR_BAD_CAN_FRAME,

	ICAN_ERR_QUOTA_EXCEEDED,
	ICAN_ERR_LOGIC,                ///< May be returned by a service if it goes wrong
	ICAN_ERR_NOT_RUNNING,
	ICAN_ERR_NOT_REGISTERED,
	ICAN_ERR_NOT_ADVERTISED,
	ICAN_ERR_IS_ADVERTISED,
	ICAN_ERR_IS_PUBLISHED,
	ICAN_ERR_IS_SUBSCRIBED,
	ICAN_INF_NO_NODEID_YET,
	ICAN_INF_NO_DATA
} InsCanErrorCode;

#endif