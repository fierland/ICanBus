//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// ICanBaseSrv.cpp
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
#include "ICanBaseSrv.h"
#include "ican_debug.h"
#include <esp_task_wdt.h>

// defines of static elements
QList<ICanBaseSrv::_CanServiceLinks*> ICanBaseSrv::_serviceRefs;

ICanBaseSrv::ICanBaseSrv(nodeId_t nodeId, uint8_t hdwId, uint8_t swId)
{
	DPRINTINFO("START");
	ICanBeaconRequestPayload canasCfg;

	_node_id = nodeId;

	canasCfg.hardware_revision = hdwId;
	canasCfg.software_revision = swId;
	// TODO: node state
	canasCfg.node_state = 0;

	// create standard services

	_beaconService = new ICanService_beacon((ICanBase*)this, &canasCfg);
	_nodeIdService = new ICanService_getNodeId((ICanBase*)this);

	_acceptDataService = new ICanService_acceptdata((ICanBase*)this);
	_requestDataService = new ICanService_requestdata((ICanBase*)this);

	ServiceRegister(_beaconService);
	ServiceRegister(_nodeIdService);
	ServiceRegister(_acceptDataService);
	ServiceRegister(_requestDataService);

	DPRINTINFO("STOP");
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::_diffU8(uint8_t a, uint8_t b)
{
	const int d = a - b;
	if (d <= -128)
		return 256 + d;
	else if (d >= 127)
		return d - 256;
	return d;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
ICanBaseSrv::~ICanBaseSrv()
{
	delete _nodeIdService;
	delete _acceptDataService;
	delete _requestDataService;
	delete _beaconService;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ParamUnsubscribe(canbusId_t msg_id)
{
	CanasCanFrame pframe;
	_CanDataRegistration* tmpRef;
	int	listId;
	DPRINTINFO("START");

	DPRINT("removing:");
	DPRINTLN(msg_id);

	listId = _findDataRegistrationInList(msg_id);
	if (listId == -1)
		return -ICAN_ERR_NO_SUCH_ENTRY;

	/// TODO: send notification to master to stop sending data
	_requestDataService->Request(msg_id, _masterNodeId, 0);

	tmpRef = _listCanDataRegistrations[listId];
	_listCanDataRegistrations.clear(listId);
	delete tmpRef;

	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::_handleReceivedParam(CanasMessage* pframe, int subId, long timestamp)
{
	_CanDataRegistration* curParm;
	DLPRINTINFO(1, "START");

	curParm = _listCanDataRegistrations[subId];

	// check if new message
	/// positive if a > b
	int msg_diff = _diffU8(pframe->message_code, curParm->last_message_code);

	curParm->tsReceived = timestamp;

	if (msg_diff <= 0)
	{
		DLPRINT(1, "No new message code Last=");
		DLPRINT(1, curParm->last_message_code);
		DLPRINT(1, " new=");
		DLPRINTLN(1, pframe->message_code);

		return 0;
	}
	curParm->last_message_code = pframe->message_code;

	if (curParm->data.container.FLOAT != pframe->data.container.FLOAT)
	{
		curParm->data.container.FLOAT = pframe->data.container.FLOAT;
		curParm->lastVal = curParm->data.container.FLOAT;
		curParm->tsChanged = timestamp;
		/// TODO: check type of parameter and use corect cast
		_updateItem(curParm);
	}

	DLPRINTINFO(1, "STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::_handleReceivedService(CanasMessage* pmsg, int subId, long timestamp)
{
	_CanServiceLinks* curParm;
	int res = 0;

	DLPRINTINFO(1, "START");
	DLVARPRINTLN(1, "SubId=", subId);

	curParm = _serviceRefs[subId];

	if (curParm->service == NULL)
	{
		DLPRINTLN(0, "Badservice pointer");
		return -1;
	}

	// check if new message

	if (pmsg->can_id % 2)
	{		// check if response or reply
		res = curParm->service->ProcessFrame(pmsg);
	}
	else
	{
		res = curParm->service->Response(pmsg);
	}

	DLPRINTINFO(1, "STOP");
	return res;
}
//-------------------------------------------------------------------------------------------------------------------
// register new service on bus
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServiceRegister(ICanService* newService)
{
	DPRINTINFO("START");

	if (newService == NULL)
		return -ICAN_ERR_ARGUMENT;

	uint8_t service_code = newService->serviceId();

	DPRINT("adding:");
	DPRINTLN(service_code);

	_CanServiceLinks* newRef;

	int curRecord = _findServiceInList(service_code);

	if (curRecord != -1)
		return -ICAN_ERR_ENTRY_EXISTS;

	//not found so create new item
	DPRINTLN("New record");
	newRef = new _CanServiceLinks;
	DPRINTLN("added object");
	_serviceRefs.push_back(newRef);
	DPRINTLN("in list");
	newRef->linkId = _serviceRefs.size();
	DPRINTLN("id adeded");
	newRef->canServiceCode = service_code;
	DPRINTLN("type added");
	newRef->service = newService;

	DPRINTINFO("STOP");

	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// unregister a service
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServiceUnregister(service_channel_t service_code)
{
	DPRINTINFO("START");
	DPRINT("removing:");
	DPRINTLN(service_code);

	int curService = _findServiceInList(service_code);
	_CanServiceLinks* tmpRef;
	CanasCanFrame pframe;

	if (curService == -1)
		return -ICAN_ERR_NO_SUCH_ENTRY;

	/// TODO: send notification to master to stop sending data

	tmpRef = _serviceRefs[curService];
	_serviceRefs.clear(curService);

	delete tmpRef;

	DPRINTINFO("STOP");

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// subscribe to a service
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServiceSubscribe(service_channel_t serviceChannel, bool replyOnly)
{
	// check if we have registered the service
	int curRecord = -1;
	_CanServiceLinks* newRef;

	DPRINTINFO("START");
	// find in array
	DPRINT("adding:"); DPRINTLN(serviceChannel);

	// check if we know this param id
	curRecord = _findServiceInList(serviceChannel);

	if (curRecord == -1)
	{
		DPRINT("Not subscribed");
		return -ICAN_ERR_NO_SUCH_ENTRY;
	}

	if (_serviceRefs[curRecord]->isSubscribed)
	{
		DPRINTLN("!!Already subscribed");
		return -ICAN_ERR_IS_SUBSCRIBED;
	}

	_serviceRefs[curRecord]->isSubscribed = true;
	_serviceRefs[curRecord]->isReplyOnly = replyOnly;
	uint16_t canId = serviceChannelToMessageID(serviceChannel, !replyOnly);
	DPRINT("Set can filter to:"); DPRINTLN(canId);

	// set filter in canbus to capture this element
	if (_canBus.setFilter(canId, _callBackService) != 0)
	{
		DPRINT("Error creating filter");
		return -ICAN_ERR_DRIVER;
	};

	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// advertise a service
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServiceAdvertise(service_channel_t service_code, long interval)
{
	int curRecord = -1;
	_CanServiceLinks* newRef;

	DPRINTINFO("START");
	/// TODO: find in array

#if DEBUG_LEVEL > 0
	Serial.print("CAN srv advertise:"); Serial.println(service_code);
#endif

	// check if we know this param id
	curRecord = _findServiceInList(service_code);

	if (curRecord == -1)
	{
		// not found so we can not advertise
		DPRINTLN("Not found");
		return -ICAN_ERR_NO_SUCH_ENTRY;
	}
	else
	{
		//not found so set status to advertise
		DPRINTLN("Found so set to advertise");
		_serviceRefs[curRecord]->isAdvertised = true;
		_serviceRefs[curRecord]->maxIntervalAdvertiseMs = interval;
		_serviceRefs[curRecord]->tsAdvertise = 0;		// so will triger in next update loop
	}

	DPRINTINFO("STOP");
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
// advertise a service
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServicePublish(service_channel_t service_code)
{
	DPRINTINFO("START");
	int curRecord = -1;
	int retVal = 0;

	DPRINT("publish");
	DPRINTLN(service_code);

	// check if we know this param id
	curRecord = _findServiceInList(service_code);

	if (curRecord == -1)
	{
		// not found so we can not advertise
		DPRINTLN("Not found");
		retVal = -ICAN_ERR_NO_SUCH_ENTRY;
	}
	else
	{
		_serviceRefs[curRecord]->isPublished = true;
		retVal = ServicePublish(curRecord);
	}

	DPRINTINFO("STOP");
	return retVal;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServicePublish(int currentRecord)
{
	DPRINTINFO("START");
	unsigned long timestamp = Timestamp();

#if DEBUG_LEVEL > 0
	if (_serviceRefs[currentRecord]->canServiceCode != 0)
	{
		Serial.print("CAN srv advertise:"); Serial.println(_serviceRefs[currentRecord]->canServiceCode);
	}
#endif

	if (!_serviceRefs[currentRecord]->isAdvertised)
	{
#if DEBUG_LEVEL > 0
		Serial.println("NOT advertised:");
#endif
		DPRINTLN("!!! not avertised");
		return -ICAN_ERR_NOT_ADVERTISED;
	}
	_serviceRefs[currentRecord]->service->Request(timestamp);
	_serviceRefs[currentRecord]->tsAdvertise = timestamp;

	DPRINTINFO("START");
	return ICAN_ERR_OK;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void  IRAM_ATTR ICanBaseSrv::CallBackData(CAN_FRAME * pmessage)
{
	CanasCanFrame frame;
	_CanDataRegistration* tmpRef;
	int	listId;
	float newValue;
	ulong timeout;

	DLPRINTINFO(2, "START");

	CANdriver::can2areo(&frame, pmessage);

	listId = _findDataRegistrationInList(frame.id);

#if DEBUG_LEVEL > 1
	Serial.println("");
	Serial.print("###CAN callback:");
	Serial.print(frame.id);
	Serial.print(":found=");
	Serial.print(listId);
	DumpCanFrame(&frame);
#endif

	if (listId != -1)
	{
		tmpRef = _listCanDataRegistrations[listId];

		if (tmpRef->lock2)
			return;

		tmpRef->lock = true;
		CanasMessage msg;

		CANdriver::frame2msg(&msg, &frame);
		// test for data type

		newValue = 0;

		switch (msg.data.type)
		{
		case CANAS_DATATYPE_FLOAT:
			newValue = msg.data.container.FLOAT;
			break;
		case CANAS_DATATYPE_USHORT:
			newValue = msg.data.container.USHORT;
			break;
		default:
			DLPRINT(0, "!! Unhandeled datatype");
		}

		tmpRef->tsReceived = CANdriver::timeStamp();
		DLVARPRINT(2, "New Timestamp recived=", tmpRef->tsReceived);
		DLVARPRINT(2, " Old val=", tmpRef->lastVal);
		DLVARPRINTLN(2, " new val=", newValue);

		if (tmpRef->lastVal != newValue)
		{
			tmpRef->lastVal = newValue;
			tmpRef->doUpdate = true;
			tmpRef->tsChanged = tmpRef->tsReceived;
		}
		tmpRef->lock = false;
	}

	DLPRINTINFO(2, "STOP");
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void IRAM_ATTR ICanBaseSrv::CallBackService(CAN_FRAME * canFrame)
{
	CanasCanFrame frame;
	CanasMessage msg;
	CanasDataContainer canData;
	bool isRequest;

	DLPRINTINFO(5, "START");

	CANdriver::can2areo(&frame, canFrame);
	CANdriver::frame2msg(&msg, &frame);

#if DEBUG_LEVEL > 0
	//if (msg.service_code != 0)
	//{
	Serial.println("");
	Serial.print("###CAN service callback:");
	DumpCanFrame(&frame);
	DumpMessage(&msg);
	//}
#endif

	DPRINT("from node:"); DPRINT(msg.node_id); DPRINT(": Service="); DPRINTLN(msg.service_code);
	DPRINTLN(msg.can_id % 2);

	isRequest = ((msg.can_id % 2) == 0);

	if (isRequest && (msg.node_id != _node_id) && (msg.service_code != 0))
	{
		// request not not adresed to this node and not generic channel so ignore
		DPRINTLN("request not for me");
		return;
	}

	if (!isRequest)
	{
		// check if we have send request
		int i = 0;
		while ((i < _serviceReqQueue.length()) && (_serviceReqQueue[i]->node_id != msg.node_id) && \
			(_serviceReqQueue[i]->service_channel != msg.service_code))
		{
			DPRINT("item :"); DPRINT(i); DPRINT(": mode_id:"); DPRINT(_serviceReqQueue[i]->node_id); DPRINT(": srv:"); DPRINTLN(_serviceReqQueue[i]->service_channel);
			i++;
		}
		DPRINT("item found=:"); DPRINTLN(i);

		if ((_serviceReqQueue.length() == 0) || (i > _serviceReqQueue.length()))
		{
			DPRINTLN("reply not for me");
			return;
		}
	}

	int curRecord = _findServiceInList(msg.service_code);
	if (curRecord != -1)
	{
		_handleReceivedService(&msg, curRecord, millis());
	}
	else
	{
		DPRINT("CB foreign service msgid="); DPRINT(msg.service_code); DPRINT(" datatype="); DPRINTLN(msg.data.type);
	}

	DLPRINTINFO(5, "STOP");
}
//-------------------------------------------------------------------------------------------------------------------
// helper function to find a item i the list of  subscribed items
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::_findServiceInList(service_channel_t toFind)
{
	int curRecord = -1;
	_CanServiceLinks* tmpRef;

	DLPRINTINFO(2, "START");
	DLVARPRINT(3, "find in list:", toFind); DLVARPRINTLN(3, "items in list:", _serviceRefs.size());

	int rsize = _serviceRefs.size();
	for (int i = 0; i < rsize; i++)
	{
		DLVARPRINTLN(3, "check item:", i);
		DLVARPRINT(3, "Compaire:", toFind); DLVARPRINTLN(3, ":with:", tmpRef->canServiceCode);
		tmpRef = _serviceRefs[i];
		if (toFind == tmpRef->canServiceCode)
		{
			curRecord = i;
			break;
		}
	}

	DLVARPRINTLN(3, "find in list done:", curRecord);
	DLPRINTINFO(2, "STOP");

	return curRecord;
}