//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// ICanBus.cpp
// Main Can protocol class base stuff.
//
// VERSION HISTORY:
//
//==================================================================================================
// A light implementation of the CAN Aerospace protocol to manage simulated instruments.
//	This code is in the public domain.
//
// Thanks to mjs513/CANaerospace (Pavel Kirienko, 2013 (pavel.kirienko@gmail.com))
//-------------------------------------------------------------------------------------------------------------------

#include "ICanBus.h"
#include "ican_debug.h"
#include "CanasId.h"

//-------------------------------------------------------------------------------------------------------------------
// constructor
//-------------------------------------------------------------------------------------------------------------------
ICanBus::ICanBus(uint8_t nodeId, uint8_t hdwId, uint8_t swId, void(*myCallBack)(CAN_FRAME*), void(*myCallBackService)(CAN_FRAME*)) : ICanBaseSrv(nodeId, hdwId, swId)
{
	DPRINTINFO("START");

	if (myCallBack != NULL)
		_callBackData = myCallBack;
	else
		_callBackData = CallBack;

	if (myCallBackService != NULL)
		_callBackService = myCallBackService;
	else
		_callBackService = CallBackService;

	ServiceSubscribe(ICAN_SRV_BEACON); // listen to replys
	ServiceAdvertise(ICAN_SRV_BEACON); // start sending requests
	ServiceSubscribe(ICAN_SRV_GETNODEID, false);
	ServiceSubscribe(ICAN_SRV_ACCEPT_CANDATA, false);
	ServiceSubscribe(ICAN_SRV_REQUEST_CANDATA, false);

	// start listening to power state info from central

	DPRINTINFO("STOP");
}
//-------------------------------------------------------------------------------------------------------------------
// destructor
//-------------------------------------------------------------------------------------------------------------------
ICanBus::~ICanBus()
{}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::start(int speed)
{
	DPRINTINFO("START");
	_running = true;
	_canBus.start(speed);

	ParamAdvertise(CANAS_NOD_USR_AVIONICS_ON, true, 20);

	DPRINTINFO("STOP");
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// update canbus check for new input
// TODO: clean up subscribed data elements will use CAN filter as will subscribed services
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::Update()
{
	const uint64_t timestamp = _canBus.timeStamp();
	CanasCanFrame frame;
	CanasMessage msg;
	MessageGroup msggroup = MSGGROUP_WTF;

	int ret = 0;
	int curRecord = -1;

#if DEBUG_LEVEL > 10
	DPRINTINFO("START");
#endif

	if (!_running)
	{
		DPRINTINFO("!!! NOT Running");
		return -ICAN_ERR_NOT_RUNNING;
	}
	if (_canBus.receive(&frame) == 0)
	{
		DPRINT("********** UPDATE received:");
		DumpCanFrame(&frame);

		if (frame.dlc < 4 || frame.dlc>8)
		{
			DPRINTLN("update: data size incorrect");
			ret = -ICAN_ERR_BAD_CAN_FRAME;
		}

		if (frame.id & CANAS_CAN_FLAG_RTR)
		{
			DPRINTLN("frameparser: RTR flag is not allowed");
			return -ICAN_ERR_BAD_CAN_FRAME;
		}

		DumpCanFrame(&frame);
		CANdriver::frame2msg(&msg, &frame);
		DumpMessage(&msg);

		DPRINT("check group for="); DPRINTLN(msg.can_id);

		msggroup = _detectMessageGroup(msg.can_id);
		if (msggroup == MSGGROUP_WTF)
		{
			DPRINTLN("update: failed to detect the message group");
			ret = -ICAN_ERR_BAD_MESSAGE_ID;
		}

		if (msggroup == MSGGROUP_PARAMETER)
		{
			DPRINT("CANaero:update record=");
			DPRINTLN(msg.service_code);

			curRecord = _findDataRegistrationInList(msg.can_id);
			if (curRecord != -1)
			{
				_handleReceivedParam(&msg, curRecord, timestamp);
			}
			else
			{
				DPRINT("foreign param msgid=");
				DPRINT(msg.service_code);
				DPRINT(" datatype=");
				DPRINTLN(msg.data.type);
			}
		}

		else if (msggroup == MSGGROUP_SERVICE)
		{
			curRecord = _findServiceInList(msg.service_code);
			if (curRecord != -1)
			{
				_handleReceivedService(&msg, curRecord, timestamp);
			}
			else
			{
				DPRINT("U foreign service msgid="); DPRINT(msg.service_code); DPRINT(" datatype="); DPRINTLN(msg.data.type);
			}
		}
	}

	// TODO: validate if all data requests are answered other wise resend
	for (int i = 0; i < _listCanDataRegistrations.size(); i++)
	{
		_CanDataRegistration* thisReg = _listCanDataRegistrations[i];

		if (thisReg->isSubscribed)
		{
#if DEBUG_LEVEL > 1
			DPRINT("@@last timestamp="); DPRINT(thisReg->timestamp);
			DPRINT(":Now=");			DPRINT((long)timestamp);
			DPRINT(":Target="); 		DPRINTLN(thisReg->timestamp + CANAS_DEFAULT_REPEAT_TIMEOUT_USEC);
#endif
			if ((!thisReg->gotReply) &&
				((timestamp - thisReg->timestamp) > CANAS_DEFAULT_REPEAT_TIMEOUT_USEC))

			{
				thisReg->timestamp = timestamp;
				_requestDataService->Request(thisReg->canAreoId, XI_Base_NodeID);
			};

			if (thisReg->doUpdate)  //update flag set during callback
			{
				DPRINT("Do update value:"); DPRINTLN(i);
				_updateItem(thisReg);
				thisReg->doUpdate = false;
			}
		}

		if (thisReg->isAdvertised)
		{
			// loop thru all advertised param and check if we need to resend anny

#if DEBUG_LEVEL > 1
			DPRINT("@@ref=");		DPRINT(thisReg->canAreoId);
			DPRINT(":last ts=");	DPRINT(thisReg->tsPublish);
			DPRINT(":interval=");	DPRINT(thisReg->maxIntervalUs);
			DPRINT(":Now=");		DPRINT((long)timestamp);
			DPRINT(":Target=");		DPRINTLN(thisReg->timestamp + thisReg->maxIntervalUs);
#endif
			if ((thisReg->timestamp + thisReg->maxIntervalUs) < timestamp)
			{
				DPRINT("Do publish parameter:"); DPRINTLN(i);
				ParamPublish(i);
			}
		}
	};
	// loop tru all advertised services if they need to send a message

	for (int i = 0; i < _serviceRefs.size(); i++)
	{
#if DEBUG_LEVEL > 4
		DPRINT("@@srv=");	DPRINT(_serviceRefs[i]->canServiceCode);
		DPRINT(":flag=");	DPRINT(_serviceRefs[i]->isAdvertised);
		DPRINT(":last ts="); DPRINT(_serviceRefs[i]->tsAdvertise);
		DPRINT(":interval="); DPRINT(_serviceRefs[i]->maxIntervalAdvertise);
		DPRINT(":Now=");	DPRINT((long)timestamp);
		DPRINT(":Target=");	DPRINTLN(_serviceRefs[i]->tsAdvertise + _serviceRefs[i]->maxIntervalAdvertise);
#endif
		if (_serviceRefs[i]->isAdvertised && ((_serviceRefs[i]->tsAdvertise + _serviceRefs[i]->maxIntervalAdvertise) < timestamp))
		{
			DPRINT("Do publish service:");	D1PRINTLN(i);
			ServicePublish(i);
		}
	}

#if DEBUG_LEVEL > 10
	DPRINTINFO("STOP");
#endif

	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::ParamSubscribe(canbusId_t msg_id)
{
	DPRINTINFO("START");

	DPRINT("adding:");
	DPRINTLN(msg_id);

	int curRecord = -1;
	_CanDataRegistration* newRef;

	if (_detectMessageGroup(msg_id) != MSGGROUP_PARAMETER)
	{
		DPRINTLN("!!! Bad messagegroup");
		return -ICAN_ERR_BAD_MESSAGE_ID;
	}

	curRecord = _findDataRegistrationInList(msg_id);

	if (curRecord == -1)
	{
		//not found so create new item
		DPRINTLN("New record");
		newRef = new _CanDataRegistration;
		DPRINTLN("added object");
		_listCanDataRegistrations.push_back(newRef);
		DPRINTLN("in list");
		newRef->linkId = _listCanDataRegistrations.size();
		DPRINTLN("id adeded");
		newRef->canAreoId = msg_id;
		DPRINTLN("type added");
		newRef->data.type = CANAS_DATATYPE_FLOAT;
		newRef->data.length = 4;

		// TODO: add link to XPlane element
		//newRef->indicator = newIndicator;

		// set filter in canbus to capture this element
		_canBus.setFilter(msg_id, _callBackData);
		// send notification to master to start sending data
		// we are master so we acted on request from a node to receive this item. so now we wait :-)
	}
	else
	{
		DPRINTLN("!!! Entry exists");
		return -ICAN_ERR_ENTRY_EXISTS;
	}

	DPRINTLN("record processed");

	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void ICanBus::_updateItem(_CanDataRegistration* curParm)
{
	_changeXPvalueCall(curParm->canAreoId, curParm->lastVal);
}
//-------------------------------------------------------------------------------------------------------------------
// internal call if new value from XPlane then update internal array and publish new value;
//-------------------------------------------------------------------------------------------------------------------
// TODO: Adapt to XPlane interface
int ICanBus::ParamUpdateValue(canbusId_t type, float value)
{
	int curRecord = -1;
	_CanDataRegistration* newRef;

	DPRINTINFO("START");

	curRecord = _findDataRegistrationInList(type);
	if (curRecord > -1)
	{
		newRef = _listCanDataRegistrations[curRecord];
		DPRINT("CANAero:Current Value:"); DPRINT(newRef->lastVal); DPRINT(":new value:");	DPRINTLN(value);

		newRef->timestamp = Timestamp(); // update timestamp as we have activity on this value
		if (newRef->lastVal != value)
		{
			DPRINTLN("value changed");
			newRef->lastVal = value;

			// check next action
			if (newRef->isAdvertised)
			{
				newRef->last_message_code++;
				newRef->tsPublish = newRef->timestamp;
				ParamPublish(curRecord);
			}
			if (newRef->isSubscribed)
			{
				newRef->tsReceived = newRef->timestamp;

				int setDataRefValue(uint16_t canID, long value);
			}
		}
	}

	DPRINTINFO("STOP");

	return 0;
};