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
#include "Can2XPlane.h"
#include "CANdriver.h"
#include "GenericDefs.h"

//-------------------------------------------------------------------------------------------------------------------
// constructor
//-------------------------------------------------------------------------------------------------------------------
ICanBus::ICanBus(uint8_t nodeId, uint8_t hdwId, uint8_t swId, void(*myCallBack)(CAN_FRAME*), void(*myCallBackService)(CAN_FRAME*)) : ICanBaseSrv(nodeId, hdwId, swId)
{
	DPRINTINFO("START");

	if (myCallBack != NULL)
		_callBackData = myCallBack;
	else
		_callBackData = CallBackData;

	if (myCallBackService != NULL)
		_callBackService = myCallBackService;
	else
		_callBackService = CallBackService;

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
	_canBus.start(speed, 1); // start with read process on core 1 as wifi loop will use bus 0

	//ParamRegister(CANAS_NOD_USR_AVIONICS_ON, false);

	ServiceSubscribe(ICAN_SRV_BEACON); // listen to replys
	ServiceAdvertise(ICAN_SRV_BEACON); // start sending requests
	ServiceSubscribe(ICAN_SRV_GETNODEID, false);
	ServiceSubscribe(ICAN_SRV_ACCEPT_CANDATA, false);
	ServiceSubscribe(ICAN_SRV_REQUEST_CANDATA, false);
	ParamAdvertise(CANAS_NOD_USR_AVIONICS_ON, true);

	DPRINTINFO("STOP");
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::checkAdvertisements(long timestamp)
{
	DLPRINTINFO(2, "START");

	for (int i = 0; i < _serviceRefs.size(); i++)
	{
#if DEBUG_LEVEL > 5
		Serial.print("@@srv=");	Serial.print(_serviceRefs[i]->canServiceCode);
		Serial.print(":flag=");	Serial.print(_serviceRefs[i]->isAdvertised);
		Serial.print(":last ts="); Serial.print(_serviceRefs[i]->tsAdvertise);
		Serial.print(":interval="); Serial.print(_serviceRefs[i]->maxIntervalAdvertiseMs);
		Serial.print(":Now=");	Serial.print((long)timestamp);
		Serial.print(":Target=");	Serial.println(_serviceRefs[i]->tsAdvertise + _serviceRefs[i]->maxIntervalAdvertiseMs);
#endif
		if (_serviceRefs[i]->isAdvertised && ((_serviceRefs[i]->tsAdvertise + _serviceRefs[i]->maxIntervalAdvertiseMs) < timestamp))
		{
			DLVARPRINTLN(2, "CAN Do publish service:", i);
			ServicePublish(i);
		}
	}
	DLPRINTINFO(2, "STOP");
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::checkDataRefs(long timestamp)
{
	DLPRINTINFO(2, "START");
	// TODO: validate if all data requests are answered other wise resend
	for (int i = 0; i < _listCanDataRegistrations.size(); i++)
	{
		_CanDataRegistration* thisReg = _listCanDataRegistrations[i];

		DLVARPRINT(1, "Checking:", thisReg->canAreoId);
		DLVARPRINT(1, ":2xp:", thisReg->added2Xplane);
		DLVARPRINT(1, ":sub:", thisReg->isSubscribed);
		DLVARPRINT(1, ":adv:", thisReg->isAdvertised);
		DLVARPRINT(1, ":rpl:", thisReg->gotReply);
		DLVARPRINT(1, ":upd:", thisReg->doUpdate);
		DLVARPRINTLN(1, ":recv:", thisReg->tsReceived);

		if (!thisReg->added2Xplane && thisReg->isAdvertised)
		{
			// retry adding to Xplane

			DLVARPRINTLN(0, "New add to xplane", thisReg->canAreoId);

			queueDataSetItem dataItem;
			dataItem.canId = thisReg->canAreoId;
			dataItem.interval = 1;
			dataItem.send = thisReg->isAdvertised;

			if (xQueueSendToBack(xQueueDataSet, &dataItem, 10) == pdPASS)
			{
				thisReg->added2Xplane = true;
			}
			else
			{
				DLPRINTLN(0, "Error sending item to queue");
			}
		}

		if (thisReg->isSubscribed)
		{
#if DEBUG_LEVEL > 4
			Serial.print("@@last timestamp="); Serial.print(thisReg->tsReceived);
			Serial.print(":Now=");			Serial.print((long)timestamp);
			Serial.print(":Target="); 		Serial.println(thisReg->tsReceived + CANAS_DEFAULT_REPEAT_TIMEOUT_MSEC);
#endif
			if ((!thisReg->gotReply) &&
				((timestamp - thisReg->tsReceived) > CANAS_DEFAULT_REPEAT_TIMEOUT_MSEC))

			{
				thisReg->tsReceived = timestamp;
				_requestDataService->Request(thisReg->canAreoId, XI_Base_NodeID);
			};

			if (thisReg->doUpdate)  //update flag set during callback
			{
				DLVARPRINTLN(2, "Do update value:", i);
				if (_updateItem(thisReg))
					thisReg->doUpdate = false;
			}
		}

		if (thisReg->isAdvertised)
		{
			// loop thru all advertised param and check if we need to resend anny

#if DEBUG_LEVEL >3
			Serial.print("@@ref=");		Serial.print(thisReg->canAreoId);
			Serial.print(":last ts=");	Serial.print(thisReg->tsPublish);
			Serial.print(":interval=");	Serial.print(thisReg->maxICanIntervalMs);
			Serial.print(":Now=");		Serial.print((long)timestamp);
			Serial.print(":Target=");	Serial.println(thisReg->tsPublish + thisReg->maxICanIntervalMs);
#endif
			if ((timestamp - thisReg->tsPublish) > thisReg->maxICanIntervalMs)
			{
				DLVARPRINTLN(2, "Do publish parameter:", i);
				ParamPublish(i);
			}
		}
		// loop tru all advertised services if they need to send a message
	}
	DLPRINTINFO(2, "STOP");

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// update canbus check for new input
// TODO: clean up subscribed data elements will use CAN filter as will subscribed services
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::UpdateMaster()
{
	const uint64_t timestamp = _canBus.timeStamp();
	CanasCanFrame frame;
	CanasMessage msg;
	MessageGroup msggroup = MSGGROUP_WTF;

	int ret = 0;
	int curRecord = -1;

	DLPRINTINFO(1, "START");

	if (!_running)
	{
		DLPRINTINFO(2, "!!! NOT Running");
		return -ICAN_ERR_NOT_RUNNING;
	}

	if (_canBus.receive(&frame) == 0)
	{
		DLPRINTLN(2, "<< CAN UPDATE received:");
		DLDUMP_CANFRAME(2, &frame);

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

		DLPRINT(0, "Received:")
			DLDUMP_CANFRAME(0, &frame);
		CANdriver::frame2msg(&msg, &frame);
		DLDUMP_MESSAGE(0, &msg);

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

	DLVARPRINTLN(1, "Regs checked:", _listCanDataRegistrations.size());

	// TODO: make separate TASKS
	checkDataRefs(timestamp);

	checkAdvertisements(timestamp);

	checkNewDataFromXP();

#if DEBUG_LEVEL > 10
	DPRINTINFO("STOP");
#endif

	return 0;
};

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::ParamRegister(canbusId_t msg_id, bool subscribe)
{
	DPRINTINFO("START");

#if DEBUG_LEVEL > 0
	Serial.print("Register parameter:"); Serial.println(msg_id);
#endif

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
#if DEBUG_LEVEL > 0
		Serial.println("New Record");
#endif
		newRef = new _CanDataRegistration;
		_listCanDataRegistrations.push_back(newRef);
		newRef->linkId = _listCanDataRegistrations.size();
		newRef->canAreoId = msg_id;
		newRef->data.type = CANAS_DATATYPE_FLOAT;
		newRef->data.length = 4;

		//newRef->indicator = newIndicator;

		CanasXplaneTrans* foundItem;
		foundItem = Can2XPlane::fromCan2XplaneElement(msg_id);

		if (foundItem->canasId == msg_id)
		{
#if DEBUG_LEVEL > 0
			Serial.print("xref found:"); Serial.print(foundItem->canasId); Serial.print(": interval:"); Serial.println(foundItem->canIntervalMs);
#endif
			newRef->maxICanIntervalMs = foundItem->canIntervalMs;
			newRef->data.type = foundItem->canasDataType;
			newRef->data.length = CANdriver::getDataTypeSize(foundItem->canasDataType);
		}
		else
		{
#if DEBUG_LEVEL > 0
			Serial.print("xref not found:"); Serial.println(msg_id);
#endif
		}

		// set filter in canbus to capture this element
		if (subscribe)
		{
			newRef->isSubscribed = true;
			_canBus.setFilter(msg_id, _callBackData);
		}
		else
		{
			newRef->isSubscribed = false;
		}
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
int ICanBus::_updateItem(_CanDataRegistration* curParm)
{
	queueDataItem dataItem;

	dataItem.canId = curParm->canAreoId;
	dataItem.value = curParm->lastVal;
	if (xQueueSendToBack(xQueueRREF, &dataItem, 10) == pdTRUE)
		return 0;
	else
		return -1;
}
//-------------------------------------------------------------------------------------------------------------------
// internal call if new value from XPlane then update internal array and publish new value;
//-------------------------------------------------------------------------------------------------------------------
// TODO: Adapt to XPlane interface
int ICanBus::ParamUpdateValue(canbusId_t type, float value)
{
	int curRecord = -1;
	_CanDataRegistration* newRef;

	DLPRINTINFO(2, "START");

	curRecord = _findDataRegistrationInList(type);

	if (curRecord > -1)
	{
		newRef = _listCanDataRegistrations[curRecord];
		DLVARPRINT(1, "CANAero:Current Value:", newRef->lastVal); DLVARPRINTLN(1, ":new value:", value);

		newRef->tsChanged = Timestamp(); // update timestamp as we have activity on this value
		if (newRef->lastVal != value)
		{
			DLPRINTLN(1, "Value changed");
			newRef->lastVal = value;

			// check next action
			if (newRef->isAdvertised)
			{
				newRef->last_message_code++;
				//newRef->tsPublish = newRef->timestamp;
				newRef->data.container.FLOAT = value;
				ParamPublish(curRecord);
			}
			if (newRef->isSubscribed)
			{
				newRef->tsReceived = newRef->tsChanged;
			}
		}
	}
	else
		DLPRINT(2, "!! item Value not found");

	DLPRINTINFO(2, "STOP");

	return 0;
};
//-----------------------
int ICanBus::checkNewDataFromXP()
{
	int curRec;

	queueDataItem dataItem;

	while (xQueueReceive(xQueueRREF, &dataItem, 0) == pdTRUE)
	{
		ParamUpdateValue(dataItem.canId, dataItem.value);
	};

	return 0;
}