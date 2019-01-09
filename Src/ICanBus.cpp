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
static const char *TAG = "ICanBus";
#define LOG_LOCAL_LEVEL 3

#include "ICanBus.h"
#ifdef ICAN_MASTER

#include "CanasId.h"
#include "Can2XPlane.h"
#include "CANdriver.h"
#include "ican_debug.h"
#include "esp_log.h"

//-------------------------------------------------------------------------------------------------
extern "C" void taskCanbus(void* parameter)
{
	ICanBus *thisBus = (ICanBus*)parameter;
	for (;;)
	{
		thisBus->UpdateMaster();
		delay(10);
	}
}
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
{
	// if running stop
	stop();
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::start(int speed, int core)
{
	DPRINTINFO("START");
	_running = true;
	_canBus.start(speed, core); // start with read process on core 1 as wifi loop will use bus 0

	if (_firstStart)
	{
		//ParamRegister(CANAS_NOD_USR_AVIONICS_ON, false);
		ServiceSubscribe(ICAN_SRV_BEACON); // listen to replys
		ServiceAdvertise(ICAN_SRV_BEACON); // start sending requests
		ServiceSubscribe(ICAN_SRV_GETNODEID, false);
		ServiceSubscribe(ICAN_SRV_ACCEPT_CANDATA, false);
		ServiceSubscribe(ICAN_SRV_REQUEST_CANDATA, false);
		//ParamAdvertise(CANAS_NOD_USR_AVIONICS_ON, true);
		//ParamAdvertise(201, true);   /// TEST ONLY
		//ParamAdvertise(202, true);   /// TEST ONLY
		//ParamAdvertise(203, true);   /// TEST ONLY
		_canBus.setFilter(); //then let everything else through anyway

		_firstStart = false;
	}
	// start task
	xTaskCreatePinnedToCore(taskCanbus, "taskCanBus", 10000, this, 1, &xTaskCanbus, core);
	xEventGroupSetBits(s_connection_event_group, RUNNING_CANBUS_BIT);

	if (_canRunMode == CAN_STATUS_STOPPED)
		_canRunMode = CAN_STATUS_RUNNING;
	else
		_canRunMode = CAN_STATUS_BUSSTARTED;

	DPRINTINFO("STOP");

	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------

int ICanBus::stop()
{
	if (_running)
	{
		vTaskDelete(xTaskCanbus);

		xEventGroupClearBits(s_connection_event_group, RUNNING_CANBUS_BIT);

		_canBus.stop();
		_running = false;
		_canRunMode = CAN_STATUS_STOPPED;
		return ICAN_ERR_OK;
	}
	else
		return -ICAN_ERR_NOT_RUNNING;
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::checkAdvertisements(long timestamp)
{
	DLPRINTINFO(2, "START");
	ESP_LOGV(TAG, "checking service advertisements:%d", _serviceRefs.size());

	for (int i = 0; i < _serviceRefs.size(); i++)
	{
		ESP_LOGV(TAG, "@@srv=%d %s Advertised ts=%d interval=%d now=%d target=%d", _serviceRefs[i]->canServiceCode,
			_serviceRefs[i]->isAdvertised ? "" : "NOT ", _serviceRefs[i]->maxIntervalAdvertiseMs,
			_serviceRefs[i]->maxIntervalAdvertiseMs, timestamp,
			_serviceRefs[i]->tsAdvertise + _serviceRefs[i]->maxIntervalAdvertiseMs);

		if (_serviceRefs[i]->isAdvertised && ((_serviceRefs[i]->tsAdvertise + _serviceRefs[i]->maxIntervalAdvertiseMs) < timestamp))
		{
			ESP_LOGD(TAG, "CAN Do publish service:%d", i);
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

		ESP_LOGV(TAG, "Checking:%d 2xp=%d subscribed=%d advertiseed=%d reply=%d updated=%d received=%d",
			thisReg->canAreoId, thisReg->added2Xplane, thisReg->isSubscribed, thisReg->isAdvertised,
			thisReg->gotReply, thisReg->doUpdate, thisReg->tsReceived);

		if (!thisReg->added2Xplane && thisReg->isAdvertised)
		{
			// retry adding to Xplane

			ESP_LOGD(TAG, "New add to xplane %d", thisReg->canAreoId);

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
				ESP_LOGE(TAG, "Error sending item to queue [%d]", thisReg->canAreoId);
			}
		}

		if (thisReg->isSubscribed)
		{
			ESP_LOGV(TAG, "@@last timestamp=%d now=%d target =%d", thisReg->tsReceived, timestamp,
				thisReg->tsReceived + thisReg->maxICanIntervalMs);

			if ((!thisReg->gotReply) &&
				((timestamp - thisReg->tsReceived) > thisReg->maxICanIntervalMs))

			{
				thisReg->tsReceived = timestamp;
				_requestDataService->Request(thisReg->canAreoId, XI_Base_NodeID);
			};

			if (thisReg->doUpdate)  //update flag set during callback
			{
				ESP_LOGD(TAG, "Do update value:%d", i);
				if (_updateItem(thisReg))
					thisReg->doUpdate = false;
			}
		}

		if (thisReg->isAdvertised)
		{
			// loop thru all advertised param and check if we need to resend anny
			ESP_LOGV(TAG, "@@ref=%d last ts=%d interval=%d now =%d target=%d", thisReg->canAreoId, thisReg->tsPublish,
				thisReg->maxICanIntervalMs, timestamp, thisReg->tsPublish + thisReg->maxICanIntervalMs);

			if ((timestamp - thisReg->tsPublish) > thisReg->maxICanIntervalMs)
			{
				ESP_LOGD(TAG, "Do publish parameter:%d", i);
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
	static long nextNodeCheck = 0;
	static long nextDataRefCheck = 0;

	int ret = 0;
	int curRecord = -1;

	DLPRINTINFO(1, "START");
	esp_task_wdt_reset();

	if (!_running)
	{
		ESP_LOGE(TAG, "!!! NOT Running");
		return -ICAN_ERR_NOT_RUNNING;
	}

	if (_canBus.receive(&frame) == 0)
	{
		ESP_LOGD(TAG, "<< CAN UPDATE received:");
		DLDUMP_CANFRAME(2, &frame);

		if (frame.dlc < 4 || frame.dlc>8)
		{
			ESP_LOGE(TAG, "update: data size incorrect");
			ret = -ICAN_ERR_BAD_CAN_FRAME;
		}

		if (frame.id & CANAS_CAN_FLAG_RTR)
		{
			ESP_LOGE(TAG, "frameparser: RTR flag is not allowed");
			return -ICAN_ERR_BAD_CAN_FRAME;
		}

		DLPRINT(0, "Received:");
		DLDUMP_CANFRAME(0, &frame);

		CANdriver::frame2msg(&msg, &frame);

		DLDUMP_MESSAGE(0, &msg);

		ESP_LOGV(TAG, "check group for=%d", msg.can_id);

		msggroup = _detectMessageGroup(msg.can_id);

		if (msggroup == MSGGROUP_WTF)
		{
			ESP_LOGE(TAG, "update: failed to detect the message group");
			ret = -ICAN_ERR_BAD_MESSAGE_ID;
		}

		if (msggroup == MSGGROUP_PARAMETER)
		{
			ESP_LOGD(TAG, "CANaero:update record=%d", msg.service_code);

			curRecord = _findDataRegistrationInList(msg.can_id);
			if (curRecord != -1)
			{
				_handleReceivedParam(&msg, curRecord, timestamp);
			}
			else
			{
				ESP_LOGD(TAG, "foreign param msgid=%d datatype=%d", msg.service_code, msg.data.type);
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
				ESP_LOGD(TAG, "U foreign service msgid=%d datatype=%d", msg.service_code, msg.data.type);
			}
		}
	}

	ESP_LOGV(TAG, "Regs checked:%d", _listCanDataRegistrations.size());
	esp_task_wdt_reset();

	// TODO: make separate TASKS
	checkNewDataFromXP();

	if (timestamp > nextDataRefCheck)
	{
		checkDataRefs(timestamp);
		nextDataRefCheck = timestamp + 100;
	}

	checkAdvertisements(timestamp);

	if (timestamp > nextNodeCheck)
	{
		_beaconService->checkNodes(timestamp);
		nextNodeCheck = timestamp + 10000;
	}

	return 0;
};

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::ParamRegister(canbusId_t msg_id, bool subscribe)
{
	DPRINTINFO("START");

	ESP_LOGD(TAG, "Register parameter:%d", msg_id);

	int curRecord = -1;
	_CanDataRegistration* newRef;

	if (_detectMessageGroup(msg_id) != MSGGROUP_PARAMETER)
	{
		ESP_LOGE(TAG, "!!! Bad messagegroup");
		return -ICAN_ERR_BAD_MESSAGE_ID;
	}

	curRecord = _findDataRegistrationInList(msg_id);

	if (curRecord == -1)
	{
		//not found so create new item
		ESP_LOGD(TAG, "New record");
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
			ESP_LOGD(TAG, "xref found:%d: interval:%d:", foundItem->canasId, foundItem->canIntervalMs);

			newRef->maxICanIntervalMs = foundItem->canIntervalMs;
			newRef->data.type = foundItem->canasDataType;
			newRef->data.length = CANdriver::getDataTypeSize(foundItem->canasDataType);
		}
		else
		{
			ESP_LOGD(TAG, "xref not found:%d:", msg_id);
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
		ESP_LOGD(TAG, "!!! Entry exists");
		return -ICAN_ERR_ENTRY_EXISTS;
	}
	ESP_LOGD(TAG, "record processed");

	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// send updated item to XpUDP
//-------------------------------------------------------------------------------------------------------------------
int ICanBus::_updateItem(_CanDataRegistration* curParm)
{
	queueDataItem dataItem;

	dataItem.canId = curParm->canAreoId;
	dataItem.data = curParm->data;
	ESP_LOGD(TAG, "update item [%d] [%f] [%f]",
		curParm->canAreoId, curParm->data.container.FLOAT, dataItem.data.container.FLOAT);
	if (xQueueSendToBack(xQueueDREF, &dataItem, 10) == pdTRUE)
		return 0;
	else
		return -1;
}
//-------------------------------------------------------------------------------------------------------------------
// internal call if new value from XPlane then update internal array and publish new value;
//-------------------------------------------------------------------------------------------------------------------
// TODO: Adapt to XPlane interface
int ICanBus::ParamUpdateValue(canbusId_t type, CanasMessageData data)
{
	int curRecord = -1;
	_CanDataRegistration* newRef;

	DLPRINTINFO(2, "START");

	curRecord = _findDataRegistrationInList(type);

	if (curRecord > -1)
	{
		newRef = _listCanDataRegistrations[curRecord];
		ESP_LOGV(TAG, "CANAero:Current Value:%f: Newvalue :%f:",
			newRef->data.container.FLOAT, data.container.FLOAT);

		newRef->tsChanged = Timestamp(); // update timestamp as we have activity on this value
		if (newRef->data.container.FLOAT != data.container.FLOAT)
		{
			ESP_LOGD(TAG, "Value changed from %f to %f", newRef->data.container.FLOAT,
				data.container.FLOAT);
			newRef->data.container = data.container;

			// check next action
			if (newRef->isAdvertised)
			{
				newRef->last_message_code++;
				//newRef->tsPublish = newRef->timestamp;
				newRef->data.container = data.container;
				ParamPublish(curRecord);
			}
			if (newRef->isSubscribed)
			{
				newRef->tsReceived = newRef->tsChanged;
			}
		}
	}
	else
	{
		// value not found so send message to UDP reader that we no longer need this item.
		ESP_LOGD(TAG, "!! item Value not found for %d", type);
		queueDataSetItem dataItem;
		dataItem.canId = type;
		dataItem.interval = 0;

		if (!xQueueSendToBack(xQueueDataSet, &dataItem, 10) == pdPASS)
		{
			ESP_LOGE(TAG, "Error sending item to queue [%d]", type);
		}
	}

	DLPRINTINFO(2, "STOP");

	return ICAN_ERR_OK;
};
//-----------------------
int ICanBus::checkNewDataFromXP()
{
	int curRec;

	queueDataItem dataItem;

	while (xQueueReceive(xQueueRREF, &dataItem, 0) == pdTRUE)
	{
		ESP_LOGD(TAG, "New data for %d from xplane [%f]", dataItem.canId,
			dataItem.data.container.FLOAT);
		// TODO: use data element
		ParamUpdateValue(dataItem.canId, dataItem.data);
	};

	return ICAN_ERR_OK;
}
#endif