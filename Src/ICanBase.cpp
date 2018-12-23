//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// ICanBase.cpp
// Can protocol class base stuff.
//
// VERSION HISTORY:
//
//==================================================================================================
// A light implementation of the CAN Aerospace protocol to manage simulated instruments.
//	This code is in the public domain.
//
// Thanks to mjs513/CANaerospace (Pavel Kirienko, 2013 (pavel.kirienko@gmail.com))
//-------------------------------------------------------------------------------------------------------------------

#include "ICanBase.h"
#include "ican_debug.h"
#include "Can2XPlane.h"
#include "GenericDefs.h"

// defines of static elements
QList<ICanBase::_CanDataRegistration*> ICanBase::_listCanDataRegistrations;

QList<ICanBase::_CanSrvReq*> ICanBase::_serviceReqQueue;

uint8_t	ICanBase::_node_id = 255;
bool ICanBase::_externalBusIsRunning = false;

#define RANGEINCLUSIVE(x, min, max) (((x) >= (min)) && ((x) <= (max)))
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
bool ICanBase::canasIsValidServiceChannel(service_channel_t service_channel)
{
	return
		RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_HIGH_MIN, CANAS_SERVICE_CHANNEL_HIGH_MAX) ||
		RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_LOW_MIN, CANAS_SERVICE_CHANNEL_LOW_MAX);
}

//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

MessageGroup ICanBase::_detectMessageGroup(canbusId_t id)
{
	DPRINTINFO("START");
	DPRINT("msgid="); DPRINTLN(id);

	if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_EMERGENCY_EVENT_MIN, CANAS_MSGTYPE_EMERGENCY_EVENT_MAX))
		return MSGGROUP_PARAMETER;
	DPRINTINFO("CHECK 1");
	if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_NODE_SERVICE_HIGH_MIN, CANAS_MSGTYPE_NODE_SERVICE_HIGH_MAX))
		return MSGGROUP_SERVICE;
	DPRINTINFO("CHECK 2");
	if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_USER_DEFINED_HIGH_MIN, CANAS_MSGTYPE_USER_DEFINED_HIGH_MAX))
		return MSGGROUP_PARAMETER;
	DPRINTINFO("CHECK 3");
	if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_NORMAL_OPERATION_MIN, CANAS_MSGTYPE_NORMAL_OPERATION_MAX))
		return MSGGROUP_PARAMETER;
	DPRINTINFO("CHECK 4");
	if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_USER_DEFINED_LOW_MIN, CANAS_MSGTYPE_USER_DEFINED_LOW_MAX))
		return MSGGROUP_PARAMETER;
	DPRINTINFO("CHECK 5");
	if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_DEBUG_SERVICE_MIN, CANAS_MSGTYPE_DEBUG_SERVICE_MAX))
		return MSGGROUP_PARAMETER;
	DPRINTINFO("CHECK 6");
	if (RANGEINCLUSIVE(id, CANAS_MSGTYPE_NODE_SERVICE_LOW_MIN, CANAS_MSGTYPE_NODE_SERVICE_LOW_MAX))
		return MSGGROUP_SERVICE;

	DPRINT("msggroup: failed to detect, msgid=");
	DPRINTLN(id);
	DPRINTINFO("STOP");

	return MSGGROUP_WTF;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::serviceChannelToMessageID(service_channel_t service_channel, bool isrequest)
{
	int ret;

	DPRINTINFO("START");
	if (RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_HIGH_MIN, CANAS_SERVICE_CHANNEL_HIGH_MAX))
	{
		ret = 128 + service_channel * 2;
		if (!isrequest)
			ret++;
		return ret;
	}
	if (RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_LOW_MIN, CANAS_SERVICE_CHANNEL_LOW_MAX))
	{
		service_channel -= CANAS_SERVICE_CHANNEL_LOW_MIN;
		ret = 2000 + service_channel * 2;
		if (!isrequest)
			ret++;
		return ret;
	}

	DPRINTINFO("STOP");

	return -ICAN_ERR_BAD_SERVICE_CHAN;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
ICanBase::ICanBase()
{
	//_listCanDataRegistrations.clear();

	DLPRINT(1, "Emptying Current request queue="); DLPRINTLN(1, _serviceReqQueue.length());
	_serviceReqQueue.clear();
	_listCanDataRegistrations.clear();
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
ICanBase::~ICanBase()
{}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ServiceSendRequest(const CanasMessage* pmsg, service_channel_t service_channel)
{
	DPRINTINFO("START");
	_CanSrvReq* newReq;
	int i;

	if (pmsg == NULL)
		return -ICAN_ERR_ARGUMENT;

#if DEBUG_LEVEL > 0
	if (service_channel != 0)
	{
		Serial.print("*** Service send request channel="); Serial.println(service_channel);
	}
#endif

	int msg_id = serviceChannelToMessageID(service_channel, true);
	if (msg_id < 0)
		return msg_id;

	DLDUMP_MESSAGE(1, pmsg);

	if (pmsg->node_id == _node_id)
	{ // Self-addressed requests are ridiculous
		return -ICAN_ERR_BAD_NODE_ID;
	}

#if DEBUG_LEVEL > 0
	Serial.print("New Msg Id="); Serial.println(msg_id);
	Serial.print("Current request queue="); Serial.println(_serviceReqQueue.length());
#endif

	// add into queu if not there
	for (i = 0;
		((i < _serviceReqQueue.length()) && (_serviceReqQueue[i]->node_id != pmsg->node_id) && \
		(_serviceReqQueue[i]->service_channel != service_channel))
		; i++);

	DLPRINT(1, "QSeach done="); DLPRINTLN(1, i);

	if ((_serviceReqQueue.length() == 0) || (i >= _serviceReqQueue.length()))
	{
		//not found
		DPRINTLN("Adding new request queue item");
		newReq = new _CanSrvReq;
		newReq->node_id = pmsg->node_id;
		newReq->service_channel = service_channel;
		_serviceReqQueue.push_back(newReq);
	}
	else
	{
		DPRINTLN("Updating request queue item");
		newReq = _serviceReqQueue[i];
	}

	newReq->timestamp = Timestamp();

	return _genericSend((uint16_t)msg_id, MSGGROUP_SERVICE, pmsg);

	DPRINTINFO("STOP");
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ServiceSendResponse(const CanasMessage* pmsg, service_channel_t service_channel)
{
	DPRINTINFO("START");
	if (pmsg == NULL)
		return -ICAN_ERR_ARGUMENT;

#if DEBUG_LEVEL > 0
	Serial.print("Service send response channel="); Serial.println(service_channel);
	DumpMessage(pmsg);
#endif

	CanasMessage msg = *pmsg;
	if (msg.node_id == CANAS_BROADCAST_NODE_ID)
	{
		DPRINTLN("srv response to broadcast request");
		msg.node_id = _node_id;   // Silently correct the Node ID for responses to global requests
	}

	if (msg.node_id != _node_id)  // Usage of foreign Node ID in response is against specification.
		return -ICAN_ERR_BAD_NODE_ID;

	int msg_id = serviceChannelToMessageID(service_channel, false); // Also will check validity of service_channel
	if (msg_id < 0)
		return msg_id;
	return _genericSend((uint16_t)msg_id, MSGGROUP_SERVICE, &msg);

	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void ICanBase::setExternalBusState(bool isRunning)
{
	DLPRINTINFO(0, "START");
	DLVARPRINTLN(0, "Set state to:", isRunning);
	_externalBusIsRunning = isRunning;
	_externalBusLastTs = Timestamp();
	DLPRINTINFO(0, "STOP");
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
bool ICanBase::isExternalBusRunning()
{
	return _externalBusIsRunning;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void ICanBase::setMasterNode(uint8_t masterNodeId)
{
	DPRINT("Set masternode to:"); DPRINTLN(masterNodeId);

	_masterNodeId = masterNodeId;
	_masterRunning = true;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
uint8_t ICanBase::getState()
{
	DLPRINTINFO(2, "START");
	uint8_t state = 0;
	state =
		((_running) ? ICAN_MASTER_STATUS_INTERFACE : 0) |
		((_externalBusIsRunning) ? ICAN_MASTER_STATUS_XPLANE_ACTIVE : 0) |
		((_instrumentPowerIsOn) ? ICAN_MASTER_STATUS_POWER : 0);

	DLVARPRINT(2, "NewState = ", _running);
	DLVARPRINT(2, ":", _externalBusIsRunning);
	DLVARPRINTLN(2, ":", _instrumentPowerIsOn);

	// TODO : return correct state of node
	DLPRINT(0, "Current state="); DLPRINTLN(0, state, BIN);
	DLPRINTINFO(2, "STOP");
	return state;
}
//-------------------------------------------------------------------------------------------------------------------
void ICanBase::setState(uint8_t state)
{
	DLPRINTINFO(2, "START");
	DLPRINT(0, "Setting state to:"); DLPRINTLN(0, state, BIN);

	_masterRunning = state & ICAN_MASTER_STATUS_INTERFACE;
	_externalBusIsRunning = state & ICAN_MASTER_STATUS_XPLANE_ACTIVE;
	if (_instrumentPowerIsOn != (state & ICAN_MASTER_STATUS_POWER))
	{
		_instrumentPowerIsOn = state & ICAN_MASTER_STATUS_POWER;
	}
	_externalBusLastTs = Timestamp();

	DLPRINTINFO(2, "STOP");
	return;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::stop()
{
	DPRINTINFO("START");
	_running = false;
	_canBus.stop();
	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void ICanBase::setCANPins(uint8_t pinRx, uint8_t pinTx)
{
	DPRINT("Setting CAN pins to RX:"); DPRINT(pinRx); DPRINT(" TX:"); DPRINTLN(pinTx);

	_canBus.setCANPins(pinRx, pinTx);
}
//-------------------------------------------------------------------------------------------------------------------
// set the id for this node to use
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::setNodeId(nodeId_t newID)
{
	DPRINTINFO("START");
	DPRINT("New node="); DPRINTLN(newID);
	_node_id = newID;

	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
nodeId_t ICanBase::getNodeId()
{
	return _node_id;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::_canasDataCopy(CanasMessageData* pdataTo, const CanasMessageData* pdataFrom)
{
	// TODO: check for null pointers
	pdataTo->length = pdataFrom->length;
	pdataTo->type = pdataFrom->type;
	for (int i = 0; i < 4; i++)
	{
		pdataTo->container.CHAR4[i] = pdataFrom->container.CHAR4[i];
	};
};
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void ICanBase::DumpCanFrame(const CanasCanFrame * pframe)
{
#if DEBUG_LEVEL >0

	Serial.print(": CANId=");
	if (pframe->id < 1000)
		Serial.print(" ");
	if (pframe->id & CANAS_CAN_FLAG_EFF)
	{
		Serial.print((pframe->id & CANAS_CAN_MASK_EXTID));
		Serial.print(" EXT ");
	}
	else
	{
		Serial.print((pframe->id & CANAS_CAN_MASK_STDID));
		Serial.print(" STD ");
	}

	if (pframe->id & CANAS_CAN_FLAG_RTR)
	{
		Serial.print(" RTR ");
	}
	else
	{
		Serial.print(" :DLC=");
		Serial.print(pframe->dlc);
		Serial.print(" :");
		int dlen;
		for (dlen = 0; dlen < pframe->dlc; dlen++)                         // hex bytes
		{
			if (pframe->data[dlen] < 16)
				Serial.print("0");
			Serial.print(pframe->data[dlen], HEX);
			Serial.print(":");
		}
		for (; dlen < 8; dlen++)                         // hex bytes
		{
			Serial.print("--:");
		}

		Serial.print("  \'");                           // ascii
		for (dlen = 0; dlen < pframe->dlc; dlen++)
		{
			uint8_t ch = pframe->data[dlen];
			if (ch < 0x20 || ch > 0x7E)
				ch = '.';
			Serial.print((char)ch);
		}
		for (; dlen < 8; dlen++)
		{
			Serial.print(" ");
		}
		Serial.print("\'");
	};

	Serial.println("<<");
#endif
	return;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

void ICanBase::DumpMessage(const CanasMessage * pmsg)
{
#if DEBUG_LEVEL > 0

	if (_externalBusIsRunning)
		Serial.print("*");
	else
		Serial.print("@");
	Serial.print("CANASMSG->can_id:");
	Serial.print((unsigned int)pmsg->can_id);
	Serial.print(": NodeId:");
	Serial.print((unsigned int)pmsg->node_id);
	Serial.print(": DataType:");
	Serial.print((unsigned int)pmsg->data.type);
	Serial.print(": ServiceCode:");
	Serial.print((unsigned int)pmsg->service_code);
	Serial.print(": MessageCode:");
	Serial.print((unsigned int)pmsg->message_code);
	Serial.print(": Data:");
	Serial.print((unsigned int)pmsg->data.container.UCHAR4[0]);
	Serial.print((unsigned int)pmsg->data.container.UCHAR4[1]);
	Serial.print((unsigned int)pmsg->data.container.UCHAR4[2]);
	Serial.print((unsigned int)pmsg->data.container.UCHAR4[3]);
	Serial.print(":");

	if (pmsg->data.type == CANAS_DATATYPE_FLOAT)
	{
		Serial.print(" Float=");
		Serial.print(pmsg->data.container.FLOAT);
	}
	else if (pmsg->data.type == CANAS_DATATYPE_LONG)
	{
		Serial.print(" Float=");
		Serial.print(pmsg->data.container.LONG);
	}
	else if (pmsg->data.type == CANAS_DATATYPE_USHORT)
	{
		Serial.print(" USHORT=");
		Serial.print(pmsg->data.container.USHORT);
	}

	Serial.println(":<<");
#endif
	return;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

uint64_t ICanBase::Timestamp()
{
	return _canBus.timeStamp();
}
//-------------------------------------------------------------------------------------------------------------------
// parse fram and convcer to message and id
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::_parseFrame(const CanasCanFrame* pframe, uint16_t* pmsg_id, CanasMessage* pmsg)
{
	DPRINTINFO("START");

	if (pframe->dlc < 4 || pframe->dlc > 8)
	{
		DPRINT("frameparser: bad dlc=");
		DPRINTLN(pframe->dlc);
		return -ICAN_ERR_BAD_CAN_FRAME;
	}
	if (pframe->id & CANAS_CAN_FLAG_RTR)
	{
		DPRINTLN("frameparser: RTR flag is not allowed");
		return -ICAN_ERR_BAD_CAN_FRAME;
	}

	*pmsg_id = pframe->id & CANAS_CAN_MASK_STDID;

	memset(pmsg, 0, sizeof(*pmsg));

	pmsg->node_id = pframe->data[0];
	pmsg->data.type = pframe->data[1];
	pmsg->service_code = pframe->data[2];
	pmsg->message_code = pframe->data[3];
	pmsg->data.length = pframe->dlc;

	for (int i = 0; i < 4; i++)
		pmsg->data.container.CHAR4[i] = 0;

	for (int i = 0; i < pmsg->data.length; i++)
		pmsg->data.container.CHAR4[i] = pframe->data[i + 4];

	DPRINTINFO("STOP");
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// generic function to send a message on canbus.
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::_genericSend(canbusId_t msg_id, uint8_t msggroup, const CanasMessage* pmsg)
{
	CanasCanFrame frame;
	int mkframe_result = -1;

	DLPRINTINFO(1, "START");

#if DEBUG_LEVEL >2
	Serial.print("CAN sending:");
	DumpMessage(pmsg);
#endif

	if (!_running)
		return -ICAN_ERR_NOT_RUNNING;

	mkframe_result = CANdriver::msg2frame(&frame, msg_id, pmsg);

	if (mkframe_result != 0)
	{
		DLVARPRINTLN(0, "!!CAN sending makeframe error:", mkframe_result);
		return mkframe_result;
	}

#if DEBUG_LEVEL >2
	Serial.print("CAN sending frame:");
	DumpCanFrame(&frame);
#endif

	bool sent_successfully = false;

	const int send_result = _canBus.writeMsg(&frame);
	if (send_result == ICAN_ERR_OK)
		sent_successfully = true;            // At least one successful sending is enough to return success.
	else
	{
		DLVARPRINTLN(0, "send failed: result=", send_result);
	}

	DLPRINTINFO(1, "STOP");
	return sent_successfully ? 0 : -ICAN_ERR_DRIVER;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ParamAdvertise(canbusId_t msg_id, bool addToXPlane, long intervalMs)
{
	int curRecord = -1;
	BaseType_t xHigherPriorityTaskWoken;

	_CanDataRegistration* curRef;
	MessageGroup msggroup = MSGGROUP_WTF;

	DLPRINTINFO(2, "START");
	/// TODO: find in array

	DLVARPRINTLN(1, "CAN Advertising adding:", msg_id);

	// check if valid record
	if (_detectMessageGroup(msg_id) != MSGGROUP_PARAMETER)
	{
		DLPRINTLN(0, "!!update: failed to detect the message group");
		return -ICAN_ERR_BAD_MESSAGE_ID;
	}

	// check if we know this param id
	curRecord = _findDataRegistrationInList(msg_id);

	if (curRecord != -1 && curRef->isAdvertised)
	{
		return -ICAN_ERR_IS_ADVERTISED;
	}

	if (curRecord == -1)
	{
		// not found so we need to create new item
		//not found so create new item
		DLPRINTLN(3, "New record");
		curRef = new _CanDataRegistration;
		DLPRINTLN(3, "added object");
		_listCanDataRegistrations.push_back(curRef);
		DLPRINTLN(3, "in list");
		curRef->canAreoId = msg_id;
		DLPRINTLN(3, "type added");
		// set filter in canbus to capture this element
//		_canBus.setFilter(msg_id, _callBackData);
		// send notification to master to start sending data
		CanasXplaneTrans* foundItem;
		foundItem = Can2XPlane::fromCan2XplaneElement(msg_id);

		if (foundItem->canasId == msg_id)
		{
#if DEBUG_LEVEL > 0
			Serial.print("xref found:"); Serial.print(foundItem->canasId); Serial.print(": interval:"); Serial.println(foundItem->canIntervalMs);
#endif
			curRef->maxICanIntervalMs = foundItem->canIntervalMs;
			curRef->data.type = foundItem->canasDataType;
			curRef->data.length = CANdriver::getDataTypeSize(foundItem->canasDataType);
		}
	}
	else
		curRef = _listCanDataRegistrations[curRecord];

	//curRef->maxICanIntervalMs = intervalMs;
	curRef->data.type = CANAS_DATATYPE_FLOAT;
	curRef->data.length = 4;
	curRef->isAdvertised = true;

	if (addToXPlane && !curRef->added2Xplane)
	{
		curRef->isXplane = true;

		/*
		if (_newXservicecall(msg_id) == 0)
			curRef->added2Xplane = true;
			*/
			//new meganism with inter task queueu
		queueDataSetItem newDset;

		newDset.canId = msg_id;
		newDset.interval = 1;
		xHigherPriorityTaskWoken = pdFALSE;
		if (xQueueSendToBackFromISR(xQueueDataSet, &newDset, &xHigherPriorityTaskWoken) == pdTRUE)
			curRef->added2Xplane = true;
	}

	/* Now the buffer is empty we can switch context if necessary. */
	if (xHigherPriorityTaskWoken)
	{
		/* Actual macro used here is port specific. */
		portYIELD_FROM_ISR();
	}

	DLPRINTLN(1, "Record processed");

	DLPRINTINFO(2, "STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ParamUnadvertise(canbusId_t msg_id)
{
	int curRecord = -1;
	DPRINTINFO("START");
	_CanDataRegistration* newRef;

	// check if we know this param id
	curRecord = _findDataRegistrationInList(msg_id);
	if (curRecord == -1)
	{
		return -ICAN_ERR_NO_SUCH_ENTRY;
	};

	if (_listCanDataRegistrations[curRecord]->isXplane)
	{
		BaseType_t xHigherPriorityTaskWoken;
		queueDataSetItem newDset;

		newDset.canId = _listCanDataRegistrations[curRecord]->canAreoId;
		newDset.interval = 0;
		xHigherPriorityTaskWoken = pdFALSE;

		if (xQueueSendToBackFromISR(xQueueDataSet, &newDset, &xHigherPriorityTaskWoken) == pdTRUE)
			_listCanDataRegistrations[curRecord]->added2Xplane = false;
	};
	// remove advertisement
	_listCanDataRegistrations[curRecord]->isAdvertised = false;

	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ParamPublish(canbusId_t msg_id, const CanasMessageData * pdata)
{
	int curRecord = -1;

	DLPRINTINFO(1, "START");

	// check we have advertised this one
	curRecord = _findDataRegistrationInList(msg_id);
	if (curRecord == -1)
	{
		return -ICAN_ERR_NO_SUCH_ENTRY;
	}

	// send message on bus
	_listCanDataRegistrations[curRecord]->lastVal = pdata->container.FLOAT;
	_listCanDataRegistrations[curRecord]->data.container.FLOAT = pdata->container.FLOAT;

	ParamPublish(curRecord);

	// TODO: code ParamPublish
	DLPRINTINFO(1, "STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ParamPublish(canbusId_t msg_id, float newVal)
{
	int curRecord = -1;

	DLPRINTINFO(1, "START");

	// check we have advertised this one
	curRecord = _findDataRegistrationInList(msg_id);
	if (curRecord == -1)
	{
		return -ICAN_ERR_NO_SUCH_ENTRY;
	}

	_listCanDataRegistrations[curRecord]->data.container.FLOAT = newVal;

	ParamPublish(curRecord);

	DLPRINTINFO(1, "STOP");

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ParamPublish(int curRecord)
{
	CanasMessage msg;

	DLPRINTINFO(1, "START");
	// TODO: What do we do if XP interface is down ?
	// send message on bus
	DLVARPRINTLN(1, "item value=", _listCanDataRegistrations[curRecord]->data.container.FLOAT);
	_canasDataCopy(&(msg.data), &(_listCanDataRegistrations[curRecord]->data));

	msg.message_code = _listCanDataRegistrations[curRecord]->last_message_code;
	msg.node_id = _node_id;
	msg.can_id = _listCanDataRegistrations[curRecord]->canAreoId;
	msg.service_code = _externalBusIsRunning ? 1 : 0;
	_listCanDataRegistrations[curRecord]->tsPublish = Timestamp();

	DLVARPRINTLN(1, "value=", msg.data.container.FLOAT);

	DLPRINTINFO(1, "STOP");
	return _genericSend(_listCanDataRegistrations[curRecord]->canAreoId, MSGGROUP_PARAMETER, &msg);
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

int ICanBase::ParamSubscribeCallback(canbusId_t msg_id, int status)
{
	int curRecord;
	DPRINTINFO("START");

	DPRINT("Records in list"); DPRINTLN(_listCanDataRegistrations.length());
	curRecord = _findDataRegistrationInList(msg_id);

	if (curRecord != -1)
	{
		DPRINT("Found "); DPRINTLN(curRecord);
		_listCanDataRegistrations[curRecord]->gotReply = true;
		_listCanDataRegistrations[curRecord]->replyCode = status;
		_listCanDataRegistrations[curRecord]->isSubscribed = true;
	}
	DPRINTINFO("STOP");
	return 0;
}

//===================================================================================================================
// internal helper functions
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// helper function to find a item i the list of  subscribed data items
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::_findDataRegistrationInList(uint16_t toFind)
{
	int curRecord = -1;
	_CanDataRegistration* tmpRef;

	DLPRINTINFO(6, "START");
	DLVARPRINT(6, "find in list:", toFind);
	DLVARPRINTLN(6, "items in list:", _listCanDataRegistrations.size());

	int rsize = _listCanDataRegistrations.size();
	for (int i = 0; i < rsize; i++)
	{
		tmpRef = _listCanDataRegistrations[i];
		DLVARPRINTLN(6, "check item:", i);
		DLVARPRINT(6, "Compaire:", toFind);
		DLVARPRINTLN(6, "with:", tmpRef->canAreoId);
		if (toFind == tmpRef->canAreoId)
		{
			curRecord = i;
			DLPRINTLN(6, "Found !!");
			break;
		}
	}

	DLVARPRINTLN(6, "find in list done:", curRecord);
	DLPRINTINFO(6, "STOP");

	return curRecord;
}