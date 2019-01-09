//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// CANdriver.cpp
// low level CAN driver stuff
//
// VERSION HISTORY:
//
//==================================================================================================
// A light implementation of the CAN Aerospace protocol to manage simulated instruments.
//	This code is in the public domain.
//
// Thanks to mjs513/CANaerospace (Pavel Kirienko, 2013 (pavel.kirienko@gmail.com))
//-------------------------------------------------------------------------------------------------------------------
static const char *TAG = "ICanDriver";
#define LOG_LOCAL_LEVEL 3

#include "CANdriver.h"
#include <stdlib.h>
#include <sys/config.h>
#include <sys/time.h>
#include <esp_timer.h>
#include "ican_debug.h"
#include "esp_log.h"
#include <machine/ieeefp.h>
#include "esp32-hal-gpio.h"
//#include "arduino.h"

// for now we are using only ESP32 nodes so always little endian

#define SWITCH(x)
/*
#ifdef __IEEE_BIG_ENDIAN
//#if __BYTE_ORDER__ == __BIG_ENDIAN
#define SWITCH(x)

#else

static inline void _switchByteOrder(void* ptr, int len)
{
	DPRINTLN("Swapping bytes");
	uint8_t tmp[4], *pbyte = (uint8_t*)ptr;
	if (len == 2)
	{
		memcpy(tmp, ptr, 2);
		pbyte[0] = tmp[1];
		pbyte[1] = tmp[0];
	}
	else if (len == 4)
	{
		memcpy(tmp, ptr, 4);
		pbyte[0] = tmp[3];
		pbyte[1] = tmp[2];
		pbyte[2] = tmp[1];
		pbyte[3] = tmp[0];
	}
}

#  define SWITCH(x) _switchByteOrder(&(x), sizeof(x))
#endif
*/

#define MARSHAL_RESULT_ERROR -1
#define MARSHAL_RESULT_UDEF  -2

#define IS_UDEF(type) ((type) >= CANAS_DATATYPE_UDEF_BEGIN_ && (type) <= CANAS_DATATYPE_UDEF_END_)
//-------------------------------------------------------------------------------------------------------------------
// still using
//-------------------------------------------------------------------------------------------------------------------

static int _marshal(CanasMessageData* pmsg)
{
	switch (pmsg->type)
	{
	case CANAS_DATATYPE_NODATA:
		return 0;

	case CANAS_DATATYPE_FLOAT:
		/*
		 * Target platform must support IEEE754 floats. The good news that almost every platform does that.
		 * But if some platform doesn't, proper converting from native float
		 * representation to IEEE754 (and vice versa) must be implemented somewhere here.
		 */
		 // FALLTHROUGH
	case CANAS_DATATYPE_ERROR:
	case CANAS_DATATYPE_LONG:
	case CANAS_DATATYPE_ULONG:
	case CANAS_DATATYPE_BLONG:
		SWITCH(pmsg->container.ULONG);
		return 4;

	case CANAS_DATATYPE_SHORT:
	case CANAS_DATATYPE_USHORT:
	case CANAS_DATATYPE_BSHORT:
		SWITCH(pmsg->container.USHORT);
		return 2;

	case CANAS_DATATYPE_CHAR:
	case CANAS_DATATYPE_UCHAR:
	case CANAS_DATATYPE_BCHAR:
		return 1;

	case CANAS_DATATYPE_SHORT2:
	case CANAS_DATATYPE_USHORT2:
	case CANAS_DATATYPE_BSHORT2:
		SWITCH(pmsg->container.USHORT2[0]);
		SWITCH(pmsg->container.USHORT2[1]);
		return 4;

	case CANAS_DATATYPE_CHAR4:
	case CANAS_DATATYPE_UCHAR4:
	case CANAS_DATATYPE_BCHAR4:
		return 4;

	case CANAS_DATATYPE_CHAR2:
	case CANAS_DATATYPE_UCHAR2:
	case CANAS_DATATYPE_BCHAR2:
		return 2;

	case CANAS_DATATYPE_MEMID:
	case CANAS_DATATYPE_CHKSUM:
		SWITCH(pmsg->container.MEMID);
		return 4;

	case CANAS_DATATYPE_ACHAR:
		return 1;

	case CANAS_DATATYPE_ACHAR2:
		return 2;

	case CANAS_DATATYPE_ACHAR4:
		return 4;

	case CANAS_DATATYPE_CHAR3:
	case CANAS_DATATYPE_UCHAR3:
	case CANAS_DATATYPE_BCHAR3:
	case CANAS_DATATYPE_ACHAR3:
		return 3;

	case CANAS_DATATYPE_DOUBLEH:
	case CANAS_DATATYPE_DOUBLEL:
		// See note about IEEE754 compatibility above.
		SWITCH(pmsg->container.DOUBLEL);
		return 4;

	default:
		if (IS_UDEF(pmsg->type) && pmsg->length <= 4)
			return MARSHAL_RESULT_UDEF;
		DPRINT("marshal: unknown data type ");
		DPRINT(pmsg->type);
		DPRINT(", udf_len = ");
		DPRINTLN(pmsg->length);
		return MARSHAL_RESULT_ERROR;
	}
}
//-------------------------------------------------------------------------------------------------------------------
int CANdriver::getDataTypeSize(uint8_t type)
{
	CanasMessageData msg;

	msg.type = type;

	return _marshal(&msg);
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
void printFrame(CAN_FRAME *message)
{
	ESP_LOGD(TAG, "FRAME: %3d[%2x] %s %d> :%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:",
		message->id, message->id, (message->extended) ? "X" : "S", message->length,
		message->data.byte[0], message->data.byte[1], message->data.byte[2], message->data.byte[3],
		message->data.byte[4], message->data.byte[5], message->data.byte[6], message->data.byte[7]);
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
CANdriver::CANdriver()
{}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
CANdriver::~CANdriver()
{}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void CANdriver::start(int speed, int processor)
{
	if (_pinRx != GPIO_NUM_16)
	{
		ESP_LOGD(TAG, "Set pin 16");
		pinMode(GPIO_NUM_16, GPIO_MODE_DEF_OUTPUT);
		digitalWrite(GPIO_NUM_16, LOW); //enable CAN transceiver
	}

	CAN0.setCANPins((gpio_num_t)_pinRx, (gpio_num_t)_pinTx);

	// TODO: set core for canbus
	//ESP_LOGD(TAG, "Set can core");
	if (processor > -1 && processor < 2)
		CAN0.set_processor(processor);

	ESP_LOGD(TAG, "Start can");
	CAN0.begin(speed);  // always returns 0

	//DLPRINT(1, "Set genericFilter");
	//CAN0.watchFor();
	ESP_LOGD(TAG, "CAN Start done");
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void CANdriver::stop()
{
	CAN0.disable();
	ESP_LOGD(TAG, "CAN Stopped");
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void CANdriver::setCANPins(uint8_t pinRx, uint8_t pinTx)
{
	_pinRx = pinRx;
	_pinTx = pinTx;
};

//-------------------------------------------------------------------------------------------------------------------
// write message to canbus
//-------------------------------------------------------------------------------------------------------------------

int CANdriver::writeMsg(CanasCanFrame* frame)
{
	DLPRINTINFO(4, "START");
	CAN_FRAME message;
	int res = ICAN_ERR_OK;

	if (frame == NULL)
		return -ICAN_ERR_ARGUMENT;

	ESP_LOGV(TAG, ">>Sending MSG:%d", frame->id);

	for (int i = 0; i < 8; i++)
		message.data.bytes[i] = frame->data[i];

	message.length = frame->dlc;
	ESP_LOGV(TAG, "dlc =%d", frame->dlc);

	if (frame->id & CANAS_CAN_FLAG_EFF)
	{
		message.extended = true;
	}
	else
	{
		message.extended = false;
	}
	message.id = frame->id;
	message.rtr = (frame->id & CANAS_CAN_FLAG_RTR) ? 1 : 0;

	if (message.rtr)
		ESP_LOGV(TAG, "RTR ");

	ESP_LOGV(TAG, ">>Sending CAN:");
	printFrame(&message);

	res = CAN0.sendFrame(message);
	ESP_LOGV(TAG, "result:%s", res ? "Direct" : "Queued");

	//DLVARPRINTLN(1, "Send result=", res);

	DLPRINTINFO(4, "START");
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
// set filter on canbus
//-------------------------------------------------------------------------------------------------------------------

int CANdriver::setFilter()
{
	CAN0.watchFor();
	return 0;
}

int CANdriver::setFilter(int msgID, void(*cbFunction)(CAN_FRAME *))
{
	int newMbx = -1;
	DLPRINTINFO(2, "START");

	if (cbFunction == NULL)
		ESP_LOGE(TAG, "Empty callback pointer");

	// fix endian conversion

	//SWITCH(msgID);

	newMbx = CAN0.watchFor(msgID);

	// TODO attach callback function
	if (newMbx != -1)
	{
		ESP_LOGD(TAG, "callback for:%d: filter set MbX = %d", msgID, newMbx);
		CAN0.setCallback(newMbx, cbFunction);
	}
	else
	{
		ESP_LOGE(TAG, "@@ ERROR setting filter");
		return -ICAN_ERR_DRIVER;
	}

	DLPRINTINFO(2, "STOP");
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
// TODO: CHECK
//-------------------------------------------------------------------------------------------------------------------
int64_t CANdriver::timeStamp()
{
	//DPRINTINFO("START");
	/*
	  timeval tv;
	  gettimeofday(&tv, NULL);
  DPRINT("Time sec=");
  DPRINT(tv.tv_sec);
  DPRINT(" usec=");
  DPRINTLN(tv.tv_usec);

	  //return (tv.tv_sec * 1000LL + (tv.tv_usec/1000LL) );
   */
   //return esp_timer_get_time();
	return millis();
	//DPRINTINFO("STOP");
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int _CanTryRead(CanasCanFrame * pframe)
{
	CAN_FRAME message;
	int res;
	//DPRINTINFO("START");

	memset(&message, 0, sizeof(CAN_FRAME));

	res = CAN0.read(message);
	if (res > 0)
	{
		ESP_LOGV(TAG, "Recived Msg->");
		printFrame(&message);

		CANdriver::can2areo(pframe, &message);
		return ICAN_ERR_OK;
	}
	//else
	//{
	//	ESP_LOGV(TAG, "Nothing Recived");
	//}
	//DPRINTINFO("STOP");
	return ICAN_INF_NO_DATA;
}
//-------------------------------------------------------------------------------------------------------------------
// need to cleanup with neat timer function
//-------------------------------------------------------------------------------------------------------------------

int CANdriver::receive(CanasCanFrame * pframe, unsigned int timeout_usec)
{
	int iTimer;
	int res;
	//DPRINTINFO("START");

	iTimer = millis() + timeout_usec;

	if (pframe == NULL)
		return -ICAN_ERR_ARGUMENT;

	while (iTimer < millis())
	{
		if (_CanTryRead(pframe) == 0)
		{
			ESP_LOGV(TAG, "frame found");
			return ICAN_ERR_OK;
		}

		delayMicroseconds(10);
	};

	//DPRINTINFO("STOP");
	return ICAN_INF_NO_DATA;
}
//-------------------------------------------------------------------------------------------------------------------
// recieve message from CAN bus
//-------------------------------------------------------------------------------------------------------------------
int CANdriver::receive(CanasCanFrame * pframe)
{
	//DPRINTINFO("START");

	if (pframe == NULL)
		return -ICAN_ERR_ARGUMENT;

	//DPRINTINFO("STOP");
	return _CanTryRead(pframe);
}

//-------------------------------------------------------------------------------------------------------------------
// recieve message from CAN bus
//-------------------------------------------------------------------------------------------------------------------
int CANdriver::receive(CanasMessage* pframe)
{
	CanasCanFrame mframe;
	//DPRINTINFO("START");

	if (pframe == NULL)
		return -ICAN_ERR_ARGUMENT;

	if (_CanTryRead(&mframe) == 0)
	{
		frame2msg(pframe, &mframe);
		return ICAN_ERR_OK;
	}

	//DPRINTINFO("STOP");
	return ICAN_INF_NO_DATA;
}

//-------------------------------------------------------------------------------------------------------------------
// convert Canbus message to can areo frame
//-------------------------------------------------------------------------------------------------------------------
int CANdriver::can2areo(CanasCanFrame * pframe, CAN_FRAME * message)
{
	DPRINTINFO("START");
	int i;

	if (message == NULL || pframe == NULL)
		return -ICAN_ERR_ARGUMENT;

	for (i = 0; i < message->length; i++)
		pframe->data[i] = message->data.bytes[i];

	for (; i < 8; i++)
		pframe->data[i] = 0;

	pframe->dlc = message->length;

	pframe->id = message->id;
	//SWITCH(pframe->id);

	ESP_LOGV(TAG, "Host CanbusID=%d", pframe->id);

	if (message->extended)
	{
		pframe->id = message->id & CANAS_CAN_MASK_EXTID;
		pframe->id |= CANAS_CAN_FLAG_EFF;
	}
	else
		pframe->id = message->id & CANAS_CAN_MASK_STDID;

	if (message->rtr == 1)
		pframe->id |= CANAS_CAN_FLAG_RTR;

	ESP_LOGV(TAG, "CanbusID=%d", pframe->id);

	DPRINTINFO("STOP");
	return ICAN_ERR_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// convert Canbus message to can areo message
//-------------------------------------------------------------------------------------------------------------------
int CANdriver::frame2msg(CanasMessage* pmsg, CanasCanFrame * pframe)
{
	int i;
	DPRINTINFO("START");
	if (pmsg == NULL || pframe == NULL)
		return -ICAN_ERR_ARGUMENT;

	ESP_LOGV(TAG, "fCanbusID=%d", pframe->id);
	ESP_LOGV(TAG, "fnodeID=%d", pframe->data[0]);

	pmsg->node_id = pframe->data[0];
	pmsg->data.type = pframe->data[1];
	pmsg->service_code = pframe->data[2];
	pmsg->message_code = pframe->data[3];
	pmsg->data.length = pframe->dlc;
	ESP_LOGV(TAG, "fCanbusID=%d", pframe->id);

	for (i = 0; i < 4; i++)
		pmsg->data.container.CHAR4[i] = pframe->data[i + 4];

	pmsg->can_id = pframe->id;
	ESP_LOGV(TAG, "mCanbusID=%d mNodeID=%d", pmsg->can_id, pmsg->node_id);

	DPRINTINFO("STOP");
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int CANdriver::msg2frame(CanasCanFrame * pframe, uint16_t msg_id, const CanasMessage * pmsg)
{
	memset(pframe, 0, sizeof(*pframe));

	pframe->id = msg_id & CANAS_CAN_MASK_STDID;
	pframe->dlc = sizeof(CanasMessage);
	pframe->data[0] = pmsg->node_id;
	pframe->data[1] = pmsg->data.type;
	pframe->data[2] = pmsg->service_code;
	pframe->data[3] = pmsg->message_code;

	int datalen = canasHostToNetwork(pframe->data + 4, &(pmsg->data));

	if (datalen < 0)
	{
		ESP_LOGE(TAG, "framemaker: bad data type=%d error =%d", pmsg->data.type, datalen);
		return datalen;
	};

	pframe->dlc = datalen + 4;

	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------

int CANdriver::canasHostToNetwork(uint8_t * pdata, const CanasMessageData * phost)
{
	DPRINTINFO("START");
	if (pdata == NULL || phost == NULL)
		return -ICAN_ERR_ARGUMENT;

	CanasMessageData nwk = *phost;
	const int ret = _marshal(&nwk);

	ESP_LOGV(TAG, "result:");
	for (int i = 0; i < 4; i++)
	{
		ESP_LOGV(TAG, "%d->%d:", phost->container.UCHAR4[i], nwk.container.UCHAR4[i]);
	}

	if (ret >= 0)
	{
		memcpy(pdata, &nwk.container, ret);
		return ret;
	}
	if (ret == MARSHAL_RESULT_UDEF)
	{
		memcpy(pdata, &nwk.container, nwk.length);
		return nwk.length;
	}
	DPRINTINFO("STOP");
	return -ICAN_ERR_BAD_DATA_TYPE;
}
//-------------------------------------------------------------------------------------------------------------------
// convert CanAero message to canbus message
//-------------------------------------------------------------------------------------------------------------------
int CANdriver::areo2can(CanasCanFrame * pframe, CAN_FRAME * message)
{
	if (pframe == NULL || message == NULL)
		return -ICAN_ERR_ARGUMENT;

	for (int i = 0; i < 8; i++)
		message->data.bytes[i] = pframe->data[i];

	message->length = pframe->dlc;
	if (pframe->id & CANAS_CAN_FLAG_EFF)
	{
		message->extended = true;
	}
	else
	{
		message->extended = false;
	}
	message->id = pframe->id;
	SWITCH(message->id);
	message->rtr = (pframe->id & CANAS_CAN_FLAG_RTR) ? 1 : 0;

	return ICAN_ERR_OK;
}