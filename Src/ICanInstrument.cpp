//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// ICanBus.h
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

#include "ICanInstrument.h"
#ifdef ICAN_INSTRUMENT

static const char *TAG = "ICanInstrument";
#define LOG_LOCAL_LEVEL 3

#include "ican_debug.h"
#include <Can2XPlane.h>
#include "CANdriver.h"
#include "esp_log.h"

//-------------------------------------------------------------------------------------------------
extern "C" void taskCanbus(void* parameter)
{
	ICanInstrument *thisBus = (ICanInstrument*)parameter;
	for (;;)
	{
		thisBus->updateInstrument();
		delay(10);
	}
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

ICanInstrument::ICanInstrument(uint8_t hdwId, uint8_t swId, void(*myCallBack)(CAN_FRAME*), void(*myCallBackService)(CAN_FRAME*)) : ICanBaseSrv(255, hdwId, swId)
{
	TRACE_START;

	if (myCallBack != NULL)
	{
		ESP_LOGD(TAG, "set callback for data");
		_callBackData = myCallBack;
	}
	else
		_callBackData = CallBackData;

	if (myCallBackService != NULL)
	{
		ESP_LOGD(TAG, "set callback for services");
		_callBackService = myCallBackService;
	}
	else
		_callBackService = CallBackService;

	_canRunMode = CAN_STATUS_INIT;

	TRACE_STOP;
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
ICanInstrument::~ICanInstrument()
{
	stop();
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
//int ICanInstrument::_getBeacon(long timestamp)
//{
//
//}

////-------------------------------------------------------------------------------------------------------------------
////
////-------------------------------------------------------------------------------------------------------------------
//int ICanInstrument::_connect2master()
//{
//	CanasCanFrame frame;
//	int ret;
//	DLPRINTINFO(1, "START");
//	// check if we got a node_id;
//	//ESP_LOGD(TAG, "Emptying Current request queue=%d", _serviceReqQueue.length());
//	//_serviceReqQueue.clear();
//
//	ESP_LOGD(TAG, "Start loop for beacon");
//	while (_node_id == 255)
//	{
//		// presume always ESP32
//		esp_task_wdt_reset();
//
//		ret = _canBus.receive(&frame);
//		//ESP_LOGD(TAG, "Canbus return:%d", ret);
//
//		if (ret == ICAN_ERR_OK)
//		{
//			ESP_LOGD(TAG, "received:");
//			DUMP_CAN_FRAME_D(TAG, &frame);
//		}
//		//else
//		//{
//		//	ESP_LOGD(TAG, "CANREAD return->%d", ret);
//		//	//DUMP_CAN_FRAME_V(TAG, &frame);
//		//}
//
//		if (_masterNodeId != 0)
//		{
//			ESP_LOGD(TAG, "@@@@START: Asking for new nodeID=%d master=%d",
//				_node_id, _masterNodeId);
//			_nodeIdService->Request(_masterNodeId, true);
//		}
//		delay(1000);
//	}; // wait for new node id
//	ESP_LOGD(TAG, "ENd loop for beacon");
//
//	_canRunMode = CAN_STATUS_GOT_ID;
//
//	ESP_LOGD(TAG, "@@@@START: got new nodeID=%d", _node_id);
//
//	// now wait until we have a ping with master node;
//
//	while (!_masterRunning)
//	{
//		// presume always ESP32
//		esp_task_wdt_reset();
//		ESP_LOGV(TAG, "*wait");
//		delay(100);
//	}
//
//	ESP_LOGD(TAG, "@@@@START: gBeacon confirmed");
//
//	DLPRINTINFO(1, "STOP");
//	return 0;
//}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int ICanInstrument::start(int speed, int toCore)
{
	if ((_canRunMode != CAN_STATUS_INIT) && (_canRunMode != CAN_STATUS_STOPPED))
	{
		ESP_LOGI(TAG, "bad state %d", _canRunMode);
		return -1;
	}

	ESP_LOGI(TAG, "@@@@START CANBUS on %d", toCore);
	// create standard services
	_canBus.start(speed, toCore);  // start with read process on core 0 as arduino loop will use core 1

	if (_canRunMode == CAN_STATUS_INIT)
	{
		ESP_LOGD(TAG, "Subscribing to services");
		ServiceSubscribe(ICAN_SRV_BEACON, false);  // for this we listen to the request
		ServiceSubscribe(ICAN_SRV_GETNODEID);
		ServiceSubscribe(ICAN_SRV_ACCEPT_CANDATA);
		ServiceSubscribe(ICAN_SRV_REQUEST_CANDATA);
		_firstStart = false;
	}

	_canBus.setFilter(); //then let everything else through anyway

	// start task
	xTaskCreatePinnedToCore(taskCanbus, "taskCanBus", 10000, this, 1, &xTaskCanbus, toCore);
	xEventGroupSetBits(s_connection_event_group, RUNNING_CANBUS_BIT);

	if (_canRunMode == CAN_STATUS_STOPPED)
	{
		ESP_LOGI(TAG, "Set runmeode to CAN_STATUS_RUNNING");
		_canRunMode = CAN_STATUS_RUNNING;
	}
	else
	{
		ESP_LOGI(TAG, "Set runmeode to CAN_STATUS_BUSSTARTED");
		_canRunMode = CAN_STATUS_BUSSTARTED;
	}

	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------

int ICanInstrument::stop()
{
	if (_running)
	{
		ESP_LOGI(TAG, "@@@@STOP CANBUS");

		vTaskDelete(xTaskCanbus);

		xEventGroupClearBits(s_connection_event_group, RUNNING_CANBUS_BIT);

		_canBus.stop();
		_running = false;
		ESP_LOGD(TAG, "Set runmeode to CAN_STATUS_STOPPED");
		_canRunMode = CAN_STATUS_STOPPED;
		return ICAN_ERR_OK;
	}
	else
		return -ICAN_ERR_NOT_RUNNING;
}
//-------------------------------------------------------------------------------------------------------------------
// main proces loop
//-------------------------------------------------------------------------------------------------------------------
void ICanInstrument::updateInstrument()
{
	long timestamp = CANdriver::timeStamp();

	//ESP_LOGV(TAG, "Runmode=[%d]", _canRunMode);
	esp_task_wdt_reset();

	switch (_canRunMode)
	{
	case CAN_STATUS_STOPPED:
		// do noting as we wait for start
		break;
	case CAN_STATUS_INIT:
		// do noting as we wait for first start
		break;
	case CAN_STATUS_BUSSTARTED:
		// bus started but no connection with master yet
		//_connect2master();
		if (_masterNodeId != 0)
		{
			ESP_LOGI(TAG, "set runmode to CAN_STATUS_GOT_BEACON");
			_canRunMode = CAN_STATUS_GOT_BEACON;
			ESP_LOGD(TAG, "@@@@START: Asking for new nodeID=%d master=%d",
				_node_id, _masterNodeId);
			_nodeIdService->Request(_masterNodeId, true);
		}
		break;
	case CAN_STATUS_GOT_BEACON:
		// got a beacon but no own node id yet
		if (_node_id != 255)
		{
			ESP_LOGI(TAG, "set runmode to CAN_STATUS_GOT_ID");
			_canRunMode = CAN_STATUS_GOT_ID;
			_externalBusLastTs = timestamp;
		}
		break;
	case CAN_STATUS_GOT_ID:
		// no extra tasks we can start running
		ESP_LOGI(TAG, "set runmode to CAN_STATUS_RUNNING");
		_canRunMode = CAN_STATUS_RUNNING;
		_running = true;
		_externalBusLastTs = timestamp;
		break;
	case CAN_STATUS_RUNNING:
		ESP_LOGD(TAG, "CHECK BUS");
		for (int i = 0; (i < CAN_MAX_ITEMS_PER_CYCLE) && (_checkCanBus(timestamp) == ICAN_ERR_OK); i++);
		ESP_LOGD(TAG, "CHECK REST");

		timestamp = CANdriver::timeStamp();

		_checkNewRegistrations(timestamp);
		_checkDataRegistrations(timestamp);
		_checkAdvertisements(timestamp);

		// check if master is still active
		if ((_externalBusLastTs + CANAS_NODE_TIMEOUT_MSEC) < timestamp)
		{
			ESP_LOGE(TAG, "##### TIMEOUT ON MASTER switch to mode CAN_STATUS_BUSSTARTED ");
			_canRunMode = CAN_STATUS_BUSSTARTED; // back to looking for beacon
			_masterRunning = false;
			_externalBusIsRunning = false;
			_instrumentPowerIsOn = false;
			_masterNodeId = 0;
			_node_id = 255;
			// poweroff instrument
		}
		break;
	default:
		ESP_LOGW(TAG, "Bad runmode [%d]", _canRunMode);
	}

	return;
}

//-------------------------------------------------------------------------------------------------------------------
// TODO: change queue struct to use multiple datatypes
//-------------------------------------------------------------------------------------------------------------------
int ICanInstrument::_updateItem(_CanDataRegistration* curParm)
{
	GenericIndicator* thisIndicator;

	float newValue = 0;

	ESP_LOGI(TAG, "New value for indicator [%d] %f",
		curParm->canAreoId, curParm->data.container.FLOAT);

	queueDataItem dataItem;

	dataItem.canId = curParm->canAreoId;
	dataItem.data = curParm->data;
	if (xQueueSendToBack(xQueueRREF, &dataItem, 10) != pdTRUE)
		ESP_LOGW(TAG, "Error sending to RREF queue");

	//thisIndicator->setValue(curParm->lastVal);
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanInstrument::ParamRegister(uint16_t msg_id, int interval, bool send)
{
	TRACE_START;
	ESP_LOGI(TAG, "adding:%d for %s", msg_id, send ? "SEND" : "receive");
	delay(10);

	int curRecord = -1;
	_CanDataRegistration* newRef;

	if (_detectMessageGroup(msg_id) != MSGGROUP_PARAMETER)
	{
		ESP_LOGE(TAG, "!!! Bad messagegroup");
		return -ICAN_ERR_BAD_MESSAGE_ID;
	}

	curRecord = _findDataRegistrationInList(msg_id);

	ESP_LOGV(TAG, "CAN Param Register:%d", msg_id);

	if (curRecord == -1)
	{
		//not found so create new item
		ESP_LOGD(TAG, "New record");
		delay(10);

		newRef = new _CanDataRegistration;
		ESP_LOGV(TAG, "added object");
		_listCanDataRegistrations.push_back(newRef);
		newRef->linkId = _listCanDataRegistrations.size();
		newRef->canAreoId = msg_id;
		newRef->data.type = CANAS_DATATYPE_FLOAT;
		newRef->data.length = 4;

		// set filter in canbus to capture this element TODO: use lib internal function

		// send notification to master to start sending data
		newRef->tsChanged = Timestamp();

		CanasXplaneTrans* foundItem;
		foundItem = Can2XPlane::fromCan2XplaneElement(msg_id);

		assert(foundItem != NULL);
		if (foundItem->canasId == msg_id)
		{
			newRef->maxICanIntervalMs = foundItem->canIntervalMs;
			newRef->data.type = foundItem->canasDataType;
			newRef->data.length = CANdriver::getDataTypeSize(foundItem->canasDataType);
		};

		newRef->isSubscribed = send;

		if (_running)
		{
			ESP_LOGD(TAG, "Subscribing to dataelement");
			newRef->isFilterSet = true;
			if (_canBus.setFilter(msg_id, _callBackData) != ICAN_ERR_OK)
				ESP_LOGV(TAG, "Error in set filter");

			if (send)
				_acceptDataService->Request(msg_id, _masterNodeId, newRef->maxICanIntervalMs);
			else
				_requestDataService->Request(msg_id, _masterNodeId, newRef->maxICanIntervalMs);
		}
	}
	else
	{
		ESP_LOGW(TAG, "!!! Entry exists");
		delay(10);
		return -ICAN_ERR_ENTRY_EXISTS;
	}

	ESP_LOGV(TAG, "record processed");

	TRACE_STOP;
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// internal call if new value from master then update n internal array and call to update physical instrument.
//-------------------------------------------------------------------------------------------------------------------
//
//int ICanInstrument::_updateValue(uint16_t type, float value)
//{
//	int curRecord = -1;
//	_CanDataRegistration* newRef;
//
//	DPRINTINFO("START");
//
//	curRecord = _findDataRegistrationInList(type);
//	if (curRecord > -1)
//	{
//		newRef = _listCanDataRegistrations[curRecord];
//		ESP_LOGV(TAG, "CANAero:Current Value:%f new value:%f", newRef->lastVal, value);
//		if (newRef->lastVal != value)
//		{
//			ESP_LOGV(TAG, "value changed");
//			newRef->lastVal = value;
//			ESP_LOGV(TAG, "Check for instrument");
//			//if (newRef->indicator != NULL)
//			//	ESP_LOGV(TAG, "Call update function");
//			_updateItem(newRef);
//			//GenericIndicator* myIndicator = (GenericIndicator*)newRef->indicator;
//			//myIndicator->setValue(value);
//		}
//	}
//
//	DPRINTINFO("STOP");
//
//	return 0;
//};
//-------------------------------------------------------------------------------------------------------------------
// checj registration queu for new items
//-------------------------------------------------------------------------------------------------------------------
int ICanInstrument::_checkNewRegistrations(long ts)
{
	int curRec;

	queueDataSetItem dataItem;
	//ESP_LOGV(TAG, "Registrations waiting %d", uxQueueMessagesWaiting(xQueueDataSet));
	delay(100);

	while (xQueueReceive(xQueueDataSet, &dataItem, 5) == pdTRUE)
	{
		ESP_LOGD(TAG, "New registration request for %d from instrument interval=%d type=%s", dataItem.canId, dataItem.interval, dataItem.send ? "DREF" : "RREF");
		delay(10);
		ParamRegister(dataItem.canId, dataItem.interval, dataItem.send);
		//ParamRegister(&dataItem);
	};

	return ICAN_ERR_OK;
};
//-------------------------------------------------------------------------------------------------------------------
// update canbus check for new input
// TODO: clean up subscribed data elements will use CAN filter as will subscribed services
//-------------------------------------------------------------------------------------------------------------------
int ICanInstrument::_checkCanBus(long timestamp)
{
	int ret = ICAN_ERR_OK;
	CanasCanFrame frame;
	CanasMessage msg;
	MessageGroup msggroup = MSGGROUP_WTF;
	_CanDataRegistration *curItem = NULL;

	int curRecord = -1;

	if (_canRunMode != CAN_STATUS_RUNNING)
	{
		ESP_LOGV(TAG, "!!! not in run state");
		return -ICAN_ERR_NOT_RUNNING;
	}

	if (_canBus.receive(&frame) == ICAN_ERR_OK)
	{
		ESP_LOGI(TAG, "DATA received: [%d]", frame.id);
		DUMP_CAN_FRAME_D(TAG, &frame);

		if (frame.dlc < 4 || frame.dlc>8)
		{
			ESP_LOGW(TAG, "update: data size incorrect");
			return -ICAN_ERR_BAD_CAN_FRAME;
		}

		if (frame.id & CANAS_CAN_FLAG_RTR)
		{
			ESP_LOGW(TAG, "frameparser: RTR flag is not allowed");
			return -ICAN_ERR_BAD_CAN_FRAME;
		}

		ESP_LOGV(TAG, "To Frame");
		CANdriver::frame2msg(&msg, &frame);
		DUMP_CAN_MSG_V(TAG, &msg);

		msggroup = _detectMessageGroup(msg.can_id);
		if (msggroup == MSGGROUP_WTF)
		{
			ESP_LOGW(TAG, "update: failed to detect the message group");
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
				ESP_LOGW(TAG, "foreign param msgid=%d  datatype=%d", msg.service_code, msg.data.type);
			}
		}
		/*
		else if (msggroup == MSGGROUP_SERVICE)
		{
			curRecord = _findServiceInList(msg.service_code);
			if (curRecord != -1)
			{
				_handleReceivedService(&msg, curRecord, timestamp);
			}
			else
			{
				DPRINT("foreign service msgid="); DPRINT(msg.service_code); DPRINT(" datatype="); DPRINTLN(msg.data.type);
			}
		}
		*/
	}
	else
	{
		ret = -ICAN_INF_NO_DATA;
	}

	return ret;
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int ICanInstrument::_checkDataRegistrations(long timestamp)
{
	_CanDataRegistration *curItem = NULL;
	TRACE_START;

	ESP_LOGD(TAG, "Checking # data registrations:%d", _listCanDataRegistrations.size());

	// TODO: validate if all data requests are answered other wise resend
	for (int i = 0; i < _listCanDataRegistrations.size(); i++)
	{
		curItem = _listCanDataRegistrations[i];

		long lastRecived = (long)curItem->tsReceived;
		long diffTime = timestamp - lastRecived;

		if (curItem->isSubscribed && !curItem->isFilterSet)
		{
			ESP_LOGI(TAG, "Subscribing to dataelement");
			curItem->isFilterSet = true;
			if (_canBus.setFilter(curItem->canAreoId, _callBackData) != ICAN_ERR_OK)
				ESP_LOGV(TAG, "Error in set filter");
		}

		ESP_LOGD(TAG, "Chekcing CanId:%d Diff=%d:Timeout=%d: %sAdvertised %sSubscribed %sReply",
			curItem->canAreoId, diffTime, CANAS_DEFAULT_REPEAT_TIMEOUT_MSEC,
			curItem->isAdvertised ? "" : "NOT ", curItem->isSubscribed ? "" : "NOT ", curItem->gotReply ? "" : "NO ");

		if ((curItem->isSubscribed) && (diffTime > CANAS_DEFAULT_REPEAT_TIMEOUT_MSEC))  // always resend after timeout && (!curItem->gotReply)

		{
			ESP_LOGV(TAG, ":Now=%d:CanId:%d:last received=%d:Diff=%d:Timeout=%d: %sAdvertised %sSubscribed %sReply",
				(long)timestamp, curItem->canAreoId, lastRecived, diffTime, CANAS_DEFAULT_REPEAT_TIMEOUT_MSEC,
				curItem->isAdvertised ? "" : "NOT ", curItem->isSubscribed ? "" : "NOT ", curItem->gotReply ? "" : "NO ");

			ESP_LOGI(TAG, "@Sending new request for data item %d", curItem->canAreoId);
			curItem->tsReceived = timestamp;
			if (_requestDataService != NULL)
				_requestDataService->Request(curItem->canAreoId, XI_Base_NodeID, curItem->maxICanIntervalMs);
		};

		// TODO: Check if still needed
		if (curItem->doUpdate)
		{
			_updateItem(curItem);
			curItem->doUpdate = false;
		}

		// for published refs
		if (curItem->isAdvertised && ((curItem->tsPublish + curItem->maxICanIntervalMs) < timestamp))
		{
			ParamPublish(i);
		}
	};
	TRACE_STOP;
}
//-------------------------------------------------------------------------------------------------------------------
int ICanInstrument::_checkAdvertisements(long timestamp)
{
	// loop tru all advertised services if they need to send a message

//	ESP_LOGV(TAG, "Checking # advertisemeets :%d", _serviceRefs.size());

	for (int i = 0; i < _serviceRefs.size(); i++)
	{
		//ESP_LOGV(TAG, ":Now=%d :srv=%d flag=%d last ts=%d interval=%d Target=%d",
		//	_serviceRefs[i]->canServiceCode, _serviceRefs[i]->isAdvertised,
		//	_serviceRefs[i]->tsAdvertise, _serviceRefs[i]->maxIntervalAdvertiseMs,
		//	_serviceRefs[i]->tsAdvertise + _serviceRefs[i]->maxIntervalAdvertiseMs);

		if (_serviceRefs[i]->isAdvertised && ((_serviceRefs[i]->tsAdvertise + _serviceRefs[i]->maxIntervalAdvertiseMs) < timestamp))
		{
			ESP_LOGI(TAG, "Do publish service:%d", i);
			ServicePublish(i);
		}
	}

	return ICAN_ERR_OK;
}

#endif