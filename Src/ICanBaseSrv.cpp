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
static const char *TAG = "ICanBaseSrv";

#include "ICanBaseSrv.h"
#include "ican_debug.h"
#include <esp_task_wdt.h>
#include "esp_log.h"

// defines of static elements
QList<ICanBaseSrv::_CanServiceLinks*> ICanBaseSrv::_serviceRefs;
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
ICanBaseSrv::ICanBaseSrv(nodeId_t nodeId, uint8_t hdwId, uint8_t swId)
{
	TRACE_START;
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

	TRACE_STOP;
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
	TRACE_START;

	ESP_LOGD(TAG, "Param unsubscribe:%d", msg_id);

	listId = _findDataRegistrationInList(msg_id);
	if (listId == -1)
		return -ICAN_ERR_NO_SUCH_ENTRY;

	// send notification to master to stop sending data
	_requestDataService->Request(msg_id, _masterNodeId, 0);

	tmpRef = _listCanDataRegistrations[listId];
	_listCanDataRegistrations.clear(listId);

	delete tmpRef;

	TRACE_STOP;
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::_handleReceivedParam(CanasMessage* pframe, int subId, long timestamp)
{
	TRACE_START;
	_CanDataRegistration* curParm;

	curParm = _listCanDataRegistrations[subId];

	// check if new message
	/// positive if a > b
	int msg_diff = _diffU8(pframe->message_code, curParm->last_message_code);

	ESP_LOGD(TAG, "handeling received canid=%d", pframe->can_id);

	curParm->tsReceived = timestamp;

	if (msg_diff <= 0)
	{
		ESP_LOGD(TAG, "No new message code Last=%d new=%d", curParm->last_message_code, pframe->message_code);

		return ICAN_INF_NO_NEW_DATA;
	}

	curParm->last_message_code = pframe->message_code;

	if (curParm->data.container.LONG != pframe->data.container.LONG)
	{
		//	memcpy(curParm->data.container,pframe->data.container,sizeof(CanasMessageData));
		curParm->data.container = pframe->data.container;
		ESP_LOGD(TAG, "data changed Last=%f new=%f", curParm->data.container.FLOAT, pframe->data.container.FLOAT);
		//curParm->lastVal = curParm->data.container.FLOAT;
		curParm->tsChanged = timestamp;
		/// TODO: check type of parameter and use corect cast
		_updateItem(curParm);
	}
	TRACE_STOP;
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::_handleReceivedService(CanasMessage* pmsg, int subId, long timestamp)
{
	TRACE_START;
	_CanServiceLinks* curParm;
	int res = 0;

	ESP_LOGD(TAG, "Got service callback SubId=%d", subId);

	curParm = _serviceRefs[subId];

	if (curParm->service == NULL)
	{
		ESP_LOGE(TAG, "Badservice pointer");
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
	TRACE_STOP;
	return res;
}
//-------------------------------------------------------------------------------------------------------------------
// register new service on bus
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServiceRegister(ICanService* newService)
{
	TRACE_START;

	if (newService == NULL)
		return -ICAN_ERR_ARGUMENT;

	uint8_t service_code = newService->serviceId();

	ESP_LOGI(TAG, "Registering service:%d", service_code);

	_CanServiceLinks* newRef;

	int curRecord = _findServiceInList(service_code);

	if (curRecord != -1)
		return -ICAN_ERR_ENTRY_EXISTS;

	//not found so create new item
	ESP_LOGV(TAG, "New record");
	newRef = new _CanServiceLinks;
	assert(newRef != NULL);
	ESP_LOGV(TAG, "added object");
	_serviceRefs.push_back(newRef);
	ESP_LOGV(TAG, "in list");
	newRef->linkId = _serviceRefs.size();
	ESP_LOGV(TAG, "id adeded");
	newRef->canServiceCode = service_code;
	ESP_LOGV(TAG, "type added");
	newRef->service = newService;

	TRACE_STOP;

	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// unregister a service
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServiceUnregister(service_channel_t service_code)
{
	TRACE_START;
	ESP_LOGI(TAG, "Unregister Service:%d", service_code);

	int curService = _findServiceInList(service_code);
	_CanServiceLinks* tmpRef;
	CanasCanFrame pframe;

	if (curService == -1)
		return -ICAN_ERR_NO_SUCH_ENTRY;

	/// TODO: send notification to master to stop sending data

	tmpRef = _serviceRefs[curService];
	_serviceRefs.clear(curService);

	delete tmpRef;

	TRACE_STOP;

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

	TRACE_START;
	// find in array
	ESP_LOGI(TAG, "Subscribing to service:%d", serviceChannel);

	// check if we know this param id
	curRecord = _findServiceInList(serviceChannel);

	if (curRecord == -1)
	{
		ESP_LOGD(TAG, "Not subscribed");
		return -ICAN_ERR_NO_SUCH_ENTRY;
	}

	if (_serviceRefs[curRecord]->isSubscribed)
	{
		ESP_LOGE(TAG, "!!Already subscribed");
		return -ICAN_ERR_IS_SUBSCRIBED;
	}

	_serviceRefs[curRecord]->isSubscribed = true;
	_serviceRefs[curRecord]->isReplyOnly = replyOnly;
	uint16_t canId = serviceChannelToMessageID(serviceChannel, !replyOnly);

	// set filter in canbus to capture this element
	// debug no filter
#ifndef ICAN_MASTER
	ESP_LOGD(TAG, "Set can filter to:%d", canId);
	if (_canBus.setFilter(canId, _callBackService) != 0)
	{
		ESP_LOGE(TAG, "Error creating filter");
		return -ICAN_ERR_DRIVER;
	};
#endif

	TRACE_STOP;
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
// advertise a service
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServiceAdvertise(service_channel_t service_code, long interval)
{
	int curRecord = -1;
	_CanServiceLinks* newRef;

	TRACE_START;
	/// TODO: find in array

	ESP_LOGI(TAG, "Adveertising service:%d", service_code);

	// check if we know this param id
	curRecord = _findServiceInList(service_code);

	if (curRecord == -1)
	{
		// not found so we can not advertise
		ESP_LOGD(TAG, "Not found");
		return -ICAN_ERR_NO_SUCH_ENTRY;
	}
	else
	{
		//not found so set status to advertise
		ESP_LOGD(TAG, "Found so set to advertise");
		_serviceRefs[curRecord]->isAdvertised = true;
		_serviceRefs[curRecord]->maxIntervalAdvertiseMs = interval;
		_serviceRefs[curRecord]->tsAdvertise = 0;		// so will triger in next update loop
	}

	TRACE_STOP;
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
// advertise a service
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServicePublish(service_channel_t service_code)
{
	TRACE_START;
	int curRecord = -1;
	int retVal = 0;

	ESP_LOGI(TAG, "Publishing service %d", service_code);

	// check if we know this param id
	curRecord = _findServiceInList(service_code);

	if (curRecord == -1)
	{
		// not found so we can not advertise
		ESP_LOGD(TAG, "Not found");
		retVal = -ICAN_ERR_NO_SUCH_ENTRY;
	}
	else
	{
		_serviceRefs[curRecord]->isPublished = true;
		retVal = ServicePublish(curRecord);
	}

	TRACE_STOP;
	return retVal;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServicePublish(int currentRecord)
{
	unsigned long timestamp = Timestamp();
	ServicePublish(currentRecord, timestamp);
}
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::ServicePublish(int currentRecord, long timestamp)
{
	TRACE_START;
	ESP_LOGI(TAG, "Publishing Service:%d", _serviceRefs[currentRecord]->canServiceCode);

	if (!_serviceRefs[currentRecord]->isAdvertised)
	{
		ESP_LOGE(TAG, "!!! not avertised");
		return -ICAN_ERR_NOT_ADVERTISED;
	}
	_serviceRefs[currentRecord]->service->Request(timestamp);
	_serviceRefs[currentRecord]->tsAdvertise = timestamp;

	TRACE_STOP;
	return ICAN_ERR_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// TODO: chanfge handeling datatype pass unchanged and let instrument handle it
//-------------------------------------------------------------------------------------------------------------------
void  ICanBaseSrv::CallBackData(CAN_FRAME * pmessage)
{
	TRACE_START;
	CanasCanFrame frame;
	_CanDataRegistration* tmpRef;
	int	listId;
	float newValue;
	ulong timeout;

	CANdriver::can2areo(&frame, pmessage);

	listId = _findDataRegistrationInList(frame.id);

	ESP_LOGI(TAG, "###CAN data callback:%d :found=%d ", frame.id, listId);
	DUMP_CAN_FRAME_D(TAG, &frame);

	if (listId != -1)
	{
		tmpRef = _listCanDataRegistrations[listId];

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
			ESP_LOGE(TAG, "!! Unhandeled datatype");
		}

		tmpRef->tsReceived = CANdriver::timeStamp();
		ESP_LOGD(TAG, "New Timestamp recived=[%d] Old val=%f new val=%f", tmpRef->tsReceived, tmpRef->data.container.FLOAT, newValue);

		if (tmpRef->data.container.FLOAT != newValue)
		{
			tmpRef->data.container.FLOAT = newValue;
			tmpRef->doUpdate = true;
			tmpRef->tsChanged = tmpRef->tsReceived;
		}
	};
	TRACE_STOP;
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
void ICanBaseSrv::CallBackService(CAN_FRAME * canFrame)
{
	TRACE_START;
	CanasCanFrame frame;
	CanasMessage msg;
	CanasDataContainer canData;
	bool isRequest;

	if (canFrame->id != 128)
		ESP_LOGI(TAG, "###CAN service callback:%d", canFrame->id);
	else
		ESP_LOGV(TAG, "###CAN service callback:%d", canFrame->id);

	CANdriver::can2areo(&frame, canFrame);
	CANdriver::frame2msg(&msg, &frame);

	DUMP_CAN_FRAME_D(TAG, &frame);
	DUMP_CAN_MSG_D(TAG, &msg);

	ESP_LOGD(TAG, "from node:%d: Service=%d [%d]", msg.node_id, msg.service_code, msg.can_id % 2);

	isRequest = ((msg.can_id % 2) == 0);

	if (isRequest && (msg.node_id != _node_id) && (msg.service_code != 0))
	{
		// request not not adresed to this node and not generic channel so ignore
		ESP_LOGD(TAG, "request not for me");
		return;
	}

	if (!isRequest)
	{
		// check if we have send request
		int i = 0;
		while ((i < _serviceReqQueue.length()) && (_serviceReqQueue[i]->node_id != msg.node_id) &&
			(_serviceReqQueue[i]->service_channel != msg.service_code))
		{
			ESP_LOGD(TAG, "item :%d: mode_id:%d: srv:%d", i, _serviceReqQueue[i]->node_id, _serviceReqQueue[i]->service_channel);
			i++;
		}
		ESP_LOGD(TAG, "item found=:%d", i);

		if ((_serviceReqQueue.length() == 0) || (i > _serviceReqQueue.length()))
		{
			ESP_LOGD(TAG, "reply not for me");
			return;
		}
	}

	int curRecord = _findServiceInList(msg.service_code);
	if (curRecord != -1)
	{
		_handleReceivedService(&msg, curRecord, CANdriver::timeStamp());
	}
	else
	{
		ESP_LOGD(TAG, "CB foreign service msgid=%d datatype=%d", msg.service_code, msg.data.type);
	}
	TRACE_STOP;
}
//-------------------------------------------------------------------------------------------------------------------
// helper function to find a item i the list of  subscribed items
//-------------------------------------------------------------------------------------------------------------------
int ICanBaseSrv::_findServiceInList(service_channel_t toFind)
{
	int curRecord = -1;
	_CanServiceLinks* tmpRef;

	TRACE_START;
	ESP_LOGV(TAG, "find in list:%d items in list:%d", toFind, _serviceRefs.size());

	int rsize = _serviceRefs.size();

	for (int i = 0; i < rsize; i++)
	{
		tmpRef = _serviceRefs[i];
		ESP_LOGV(TAG, "check item:%d Compaire:%d:with:%d:", i, (int)toFind, (int)(tmpRef->canServiceCode));
		if (toFind == tmpRef->canServiceCode)
		{
			curRecord = i;
			break;
		}
	}

	ESP_LOGV(TAG, "find in list done:%d", curRecord);
	TRACE_STOP;

	return curRecord;
}
//EOF----------------------------------------------------------------------------------------------------------------