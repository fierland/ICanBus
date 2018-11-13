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

// defines of static elements
QList<ICanBase::_CanDataRegistration*> ICanBase::_listCanDataRegistrations;
QList<ICanBase::_CanSrvReq*> ICanBase::_serviceReqQueue;

uint8_t	ICanBase::_node_id = 255;

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
	DPRINTINFO("START");
	if (RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_HIGH_MIN, CANAS_SERVICE_CHANNEL_HIGH_MAX))
	{
		int ret = 128 + service_channel * 2;
		if (!isrequest)
			ret++;
		return ret;
	}
	if (RANGEINCLUSIVE(service_channel, CANAS_SERVICE_CHANNEL_LOW_MIN, CANAS_SERVICE_CHANNEL_LOW_MAX))
	{
		service_channel -= CANAS_SERVICE_CHANNEL_LOW_MIN;
		int ret = 2000 + service_channel * 2;
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
{}

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

	if (pmsg == NULL)
		return -ICAN_ERR_ARGUMENT;

	DumpMessage(pmsg);

	if (pmsg->node_id == _node_id)
	{ // Self-addressed requests are ridiculous
		return -ICAN_ERR_BAD_NODE_ID;
	}

	int msg_id = serviceChannelToMessageID(service_channel, true);
	if (msg_id < 0)
		return msg_id;

	// add into queu if not there
	int i = 0;
	while ((i < _serviceReqQueue.length()) && (_serviceReqQueue[i]->node_id != pmsg->node_id) && \
		(_serviceReqQueue[i]->service_channel != service_channel)) i++;

	if ((_serviceReqQueue.length() == 0) || (i > _serviceReqQueue.length()))
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

	DumpMessage(pmsg);

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
	_externalBusIsRunning = isRunning;
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
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
uint8_t ICanBase::getState()
{
	uint8_t state = 0;
	state =
		((_running) ? ICAN_MASTER_STATUS_INTERFACE : 0) ||
		((_externalBusIsRunning) ? ICAN_MASTER_STATUS_XPLANE_ACTIVE : 0) ||
		((_instrumentPowerIsOn) ? ICAN_MASTER_STATUS_POWER : 0);

	// TODO : return correct state of node
	DPRINT("Current state="); DPRINTLN(state);
	return state;
}

void ICanBase::setState(uint8_t state)
{
	_masterRunning = state & ICAN_MASTER_STATUS_INTERFACE;
	_externalBusIsRunning = state & ICAN_MASTER_STATUS_XPLANE_ACTIVE;
	_instrumentPowerIsOn = state & ICAN_MASTER_STATUS_POWER;

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
#ifdef MACRO_DEBUG

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
#ifdef MACRO_DEBUG

	DPRINT("CANASMSG->can_id:");
	DPRINT((unsigned int)pmsg->can_id);
	DPRINT(": NodeId:");
	DPRINT((unsigned int)pmsg->node_id);
	DPRINT(": DataType:");
	DPRINT((unsigned int)pmsg->data.type);
	DPRINT(": ServiceCode:");
	DPRINT((unsigned int)pmsg->service_code);
	DPRINT(": MessageCode:");
	DPRINT((unsigned int)pmsg->message_code);
	DPRINT(": Data:");
	DPRINT((unsigned int)pmsg->data.container.UCHAR4[0]);
	DPRINT((unsigned int)pmsg->data.container.UCHAR4[1]);
	DPRINT((unsigned int)pmsg->data.container.UCHAR4[2]);
	DPRINT((unsigned int)pmsg->data.container.UCHAR4[3]);
	DPRINT(":");

	if (pmsg->data.type == CANAS_DATATYPE_FLOAT)
	{
		DPRINT(" Float=");
		DPRINT(pmsg->data.container.FLOAT);
	}
	else if (pmsg->data.type == CANAS_DATATYPE_LONG)
	{
		DPRINT(" Float=");
		DPRINT(pmsg->data.container.LONG);
	}
	else if (pmsg->data.type == CANAS_DATATYPE_USHORT)
	{
		DPRINT(" USHORT=");
		DPRINT(pmsg->data.container.USHORT);
	}

	DPRINTLN(":<<");
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

	DPRINTINFO("START");
	DumpMessage(pmsg);

	if (!_running)
		return -ICAN_ERR_NOT_RUNNING;

	mkframe_result = CANdriver::msg2frame(&frame, msg_id, pmsg);

	if (mkframe_result != 0)
		return mkframe_result;

	DPRINT("]]]sending:");

	DumpCanFrame(&frame);

	bool sent_successfully = false;

	const int send_result = _canBus.writeMsg(&frame);
	if (send_result == ICAN_ERR_OK)
		sent_successfully = true;            // At least one successful sending is enough to return success.
	else
	{
		DPRINTLN("send failed: result="); DPRINTLN(send_result);
	}

	DPRINTINFO("STOP");
	return sent_successfully ? 0 : -ICAN_ERR_DRIVER;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ParamAdvertise(canbusId_t msg_id, bool addToXPlane, long intervalMs)
{
	int curRecord = -1;
	_CanDataRegistration* curRef;
	MessageGroup msggroup = MSGGROUP_WTF;

	DPRINTINFO("START");
	/// TODO: find in array
	DPRINT("adding:"); DPRINTLN(msg_id); DPRINT("Interval:"); DPRINTLN(intervalMs);

	// check if valid record
	if (_detectMessageGroup(msg_id) != MSGGROUP_PARAMETER)
	{
		DPRINTLN("update: failed to detect the message group");
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
		DPRINTLN("New record");
		curRef = new _CanDataRegistration;
		DPRINTLN("added object");
		_listCanDataRegistrations.push_back(curRef);
		DPRINTLN("in list");
		curRef->canAreoId = msg_id;
		DPRINTLN("type added");
		// set filter in canbus to capture this element
//		_canBus.setFilter(msg_id, _callBackData);
		// send notification to master to start sending data
	}
	else
		curRef = _listCanDataRegistrations[curRecord];

	curRef->maxIntervalUs = intervalMs * 1000;
	curRef->data.type = CANAS_DATATYPE_FLOAT;
	curRef->data.length = 4;
	curRef->isAdvertised = true;

	if (addToXPlane)
	{
		if (_newXservicecall == NULL)
		{
			DPRINTLN("@@@@@ No xservice callback");
			return -ICAN_ERR_LOGIC;
		}
		_newXservicecall(msg_id);
		curRef->isXplane = true;
	}

	D1PRINTLN("record processed");

	DPRINTINFO("STOP");
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
		_removeXservicecall(msg_id);
		_listCanDataRegistrations[curRecord]->isXplane = false;
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

	DPRINTINFO("START");

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
	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ParamPublish(canbusId_t msg_id, float newVal)
{
	int curRecord = -1;

	DPRINTINFO("START");

	// check we have advertised this one
	curRecord = _findDataRegistrationInList(msg_id);
	if (curRecord == -1)
	{
		return -ICAN_ERR_NO_SUCH_ENTRY;
	}

	_listCanDataRegistrations[curRecord]->data.container.FLOAT = newVal;

	ParamPublish(curRecord);

	DPRINTINFO("STOP");

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBase::ParamPublish(int curRecord)
{
	CanasMessage msg;

	DPRINTINFO("START");
	// TODO: What do we do if XP interface is down ?
	// send message on bus
	_canasDataCopy(&(msg.data), &(_listCanDataRegistrations[curRecord]->data));

	msg.message_code = _listCanDataRegistrations[curRecord]->last_message_code;
	msg.node_id = _node_id;
	msg.can_id = _listCanDataRegistrations[curRecord]->canAreoId;
	msg.service_code = _externalBusIsRunning ? 1 : 0;
	_listCanDataRegistrations[curRecord]->tsPublish = Timestamp();

	return _genericSend(_listCanDataRegistrations[curRecord]->canAreoId, MSGGROUP_PARAMETER, &msg);

	DPRINTINFO("STOP");
	return 0;
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

	D1PRINTINFO("START");
	D1PRINT("find in list:"); D1PRINTLN(toFind); D1PRINT("items in list:"); D1PRINTLN(_listCanDataRegistrations.size());

	int rsize = _listCanDataRegistrations.size();
	for (int i = 0; i < rsize; i++)
	{
		tmpRef = _listCanDataRegistrations[i];
		D1PRINT("check item:"); D1PRINTLN(i);
		D1PRINT("Compaire:"); D1PRINT(toFind); D1PRINT(":with:"); D1PRINTLN(tmpRef->canAreoId);
		if (toFind == tmpRef->canAreoId)
		{
			curRecord = i;
			D1PRINTLN("Found !!");
			break;
		}
	}

	D1PRINT("find in list done:");	D1PRINTLN(curRecord);
	D1PRINTINFO("STOP");

	return curRecord;
}