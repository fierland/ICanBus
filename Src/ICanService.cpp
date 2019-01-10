//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// ICanService.cpp
// can aero based seervices.
//
// VERSION HISTORY:
//
//==================================================================================================
// A light implementation of the CAN Aerospace protocol to manage simulated instruments.
//	This code is in the public domain.
//
// Thanks to mjs513/CANaerospace (Pavel Kirienko, 2013 (pavel.kirienko@gmail.com))
//-------------------------------------------------------------------------------------------------------------------
static const char *TAG = "ICanService";

#include "ICanService.h"
#include "Can2XPlane.h"
//#include "ican_debug.h"
#include "esp_system.h"
#include "esp_log.h"

QList <ICanService::_ICanNodeInfo*> ICanService::_nodeRefs;
QList <ICanService::_canitemNode*> ICanService::_canItemNodeRefs;

#define RANGEINCLUSIVE(x, min, max) ((x) >= (min) && (x) <= (max))
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanService::_serviceChannelFromMessageID(canbusId_t msg_id, bool* pisrequest)
{
	static const uint16_t REQ_MASK = ~(uint16_t)1;
	if (RANGEINCLUSIVE(msg_id, CANAS_MSGTYPE_NODE_SERVICE_HIGH_MIN, CANAS_MSGTYPE_NODE_SERVICE_HIGH_MAX))
	{
		int srvchan = ((msg_id & REQ_MASK) - 128) / 2;
		*pisrequest = !(msg_id & 1);
		return srvchan + CANAS_SERVICE_CHANNEL_HIGH_MIN;
	}
	if (RANGEINCLUSIVE(msg_id, CANAS_MSGTYPE_NODE_SERVICE_LOW_MIN, CANAS_MSGTYPE_NODE_SERVICE_LOW_MAX))
	{
		int srvchan = ((msg_id & REQ_MASK) - 2000) / 2;
		*pisrequest = !(msg_id & 1);
		return srvchan + CANAS_SERVICE_CHANNEL_LOW_MIN;
	}

	return -ICAN_ERR_BAD_MESSAGE_ID;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
ICanService::ICanService(ICanBase * CanAsBus, ICanSrvCodes serviceID)
{
	assert(CanAsBus != NULL);

	ESP_LOGD(TAG, "Adding service :%d", serviceID);
	_CanasBus = CanAsBus;
	_myServiceId = serviceID;
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
void ICanService::checkNodes(long timestamp)
{
	// cleanup for stations not responding  ?
#ifdef ICAN_MASTER
	if ((_lastCleanupTs + CANAS_NODE_CLEANUP_MSEC) < timestamp)
	{
		ESP_LOGD(TAG, "Node cleanup for %d nodes [%d]", _nodeRefs.length(), timestamp);

		for (int i = _nodeRefs.length() - 1; i >= 0; i--)
		{
			ESP_LOGD(TAG, "Current Node %d ts=%d", _nodeRefs[i]->nodeId, _nodeRefs[i]->timestamp);
			if ((_nodeRefs[i]->timestamp + CANAS_NODE_TIMEOUT_MSEC) < timestamp)
			{
				_ICanNodeInfo* thisInfoNode = _nodeRefs[i];
				// node seems dead
				ESP_LOGI(TAG, "Killing node:%d checking %d refs", thisInfoNode->nodeId, _canItemNodeRefs.length());
				// cleanup data  for node
				for (int j = 0; j < _canItemNodeRefs.length(); j++)
				{
					_subscribedNode* curNode = NULL;
					_subscribedNode* thisNode = _canItemNodeRefs[j]->firstNode;
					ESP_LOGV(TAG, "Subscriptions %d for:%d", _canItemNodeRefs[j]->subscriptions, _canItemNodeRefs[j]->canId);

					_findNode(_canItemNodeRefs[j], thisNode->nodeId, true);

					ESP_LOGD(TAG, "Subscriptions %d for:%d", _canItemNodeRefs[j]->subscriptions, _canItemNodeRefs[j]->canId);
					if (_canItemNodeRefs[j]->subscriptions <= 0)
					{
						ESP_LOGD(TAG, "Removing Advertisement for:%d", _canItemNodeRefs[j]->canId);
						_CanasBus->ParamUnadvertise(_canItemNodeRefs[j]->canId);
						_canitemNode* thisCanNode = _canItemNodeRefs[j];
						_canItemNodeRefs.clear(j);
						delete thisCanNode;
						j--;
					}
				}

				_nodeRefs.clear(i);
				delete thisInfoNode;
			}
		}
	}
#endif
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int ICanService::_findNodeInList(uint8_t toFind)
{
	int curRecord = -1;

#ifdef ICAN_MASTER
	_ICanNodeInfo* tmpRef;

	ESP_LOGV(TAG, "find in list:%d:items in list:%d", toFind, _nodeRefs.size());

	for (int i = 0; i < _nodeRefs.size(); i++)
	{
		tmpRef = _nodeRefs[i];

		ESP_LOGV(TAG, "check item:%d: Compaire:%d:with:%d:", i, toFind, tmpRef->nodeId);

		if (toFind == tmpRef->nodeId)
		{
			curRecord = i;
			break;
		}
	}

	ESP_LOGV(TAG, "find in list done:%d", curRecord);

#endif
	return curRecord;
}
//-------------------------------------------------------------------------------------------------------------------
int ICanService::_findNode(_canitemNode* myStruct, uint8_t nodeId, bool doDelete)
{
	int result = ICAN_ERR_OK;

#ifdef ICAN_MASTER
	int i = 0;
	_subscribedNode* curNode;
	_subscribedNode* prevNode;

	ESP_LOGV(TAG, "find node:", nodeId);

	if (myStruct == NULL)
		return -ICAN_ERR_ARGUMENT;

	if (myStruct->subscriptions == 0 || myStruct->firstNode == NULL)
		return -ICAN_ERR_NO_SUCH_ENTRY;

	curNode = myStruct->firstNode;
	prevNode = NULL;

	while (curNode != NULL)
	{
		if (curNode->nodeId == nodeId)
		{
			if (doDelete)
			{
				ESP_LOGD(TAG, "Deleting node");
				if (curNode->prev != NULL)
					prevNode->prev->next = curNode->next;
				else
					myStruct->firstNode = curNode->next;

				if (curNode->next != NULL)
					curNode->next->prev = curNode->prev;
				else
					myStruct->lastNode = curNode->prev;

				delete curNode;
				myStruct->subscriptions--;
			}
			return ICAN_ERR_OK;
		}
		ESP_LOGD(TAG, "Adding node");
		prevNode = curNode;
		curNode = curNode->next;
	}

	result = -ICAN_ERR_BAD_NODE_ID;

	ESP_LOGV(TAG, "find node done:%d", result);
#endif
	return result;
}
//-------------------------------------------------------------------------------------------------------------------
// helper function to find a item i the list of  subscribed items
//-------------------------------------------------------------------------------------------------------------------
int ICanService::_findInCanIdList(uint16_t toFind)
{
	int curRecord = -1;
#ifdef ICAN_MASTER
	_canitemNode* tmpRef;

	ESP_LOGV(TAG, "CAN find canid in list:%d items in list:%d", toFind, _canItemNodeRefs.size());

	//DPRINTLN(_dataReqRefs.size());

	for (int i = 0; i < _canItemNodeRefs.size(); i++)
	{
		tmpRef = _canItemNodeRefs[i];
		ESP_LOGV(TAG, "check item:%d Compaire:%d with:%d", i, toFind, tmpRef->canId);

		if (toFind == tmpRef->canId)
		{
			curRecord = i;
			break;
		}
	}

	ESP_LOGV(TAG, "find canid done:%d", curRecord);
#endif
	return curRecord;
}
//===================================================================================================================
// Beacon service class
//-------------------------------------------------------------------------------------------------------------------

ICanService_beacon::ICanService_beacon(ICanBase * CanAsBus, ICanBeaconRequestPayload * payload) : ICanService(CanAsBus, ICAN_SRV_BEACON)
{
	ESP_LOGD(TAG, "New beacon service");
	if (payload == NULL)
	{
		ESP_LOGE(TAG, "beacon srv ids init: bad payload param");
		return;
	}
	_myRequestPayload = payload;
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
ICanService_beacon::~ICanService_beacon()
{
	delete _myRequestPayload;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanService_beacon::ProcessFrame(CanasMessage * msg)
{
#ifdef ICAN_MASTER
	_ICanNodeInfo* newNode;
	int curRecord;
	uint8_t newNodeId;

	if (msg->data.type != CANAS_DATATYPE_UCHAR4)
	{
		ESP_LOGE(TAG, "!!!Beacon: Bad data type:%d", msg->data.type);
		return -ICAN_ERR_BAD_DATA_TYPE;
	}

	newNodeId = (uint8_t)msg->data.container.UCHAR4[0];
	curRecord = _findNodeInList(newNodeId);

	if (curRecord != -1)
	{
		ESP_LOGD(TAG, "CAN beacon Found ->Node:%d", newNodeId);

		newNode = _nodeRefs[curRecord];
	}
	else
	{
		ESP_LOGD(TAG, "CAN beacon Adding node:%d", newNodeId);

		ESP_LOGV(TAG, "Added Node for:%d HWid=%x SWid=%x",
			msg->node_id, msg->data.container.UCHAR4[0], msg->data.container.UCHAR4[1]);

		newNode = new _ICanNodeInfo;
		newNode->nodeId = newNodeId;
		newNode->hardware_revision = msg->data.container.UCHAR4[1];
		newNode->software_revision = msg->data.container.UCHAR4[2];
		_nodeRefs.push_back(newNode);
	};
	// TODO: neat timestamp
	newNode->timestamp = CANdriver::timeStamp();
	newNode->nodeState = msg->data.container.UCHAR4[3];
#endif
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
// send response to a identification service request`from client
//-------------------------------------------------------------------------------------------------------------------
int ICanService_beacon::Response(CanasMessage * msg)
{
#ifdef ICAN_INSTRUMENT
	// check if it is request or reply
	uint8_t myNodeId;

	if (msg == NULL)
	{
		ESP_LOGE(TAG, "Beacon srv ids response: bad msg param");
		return -ICAN_ERR_ARGUMENT;
	}

	// check if master node is correct
	//msg->node_id = msg->data.container.UCHAR4[1];
	ESP_LOGD(TAG, "New state=%x", msg->data.container.UCHAR4[0]);

	uint8_t newState = msg->data.container.UCHAR4[0];
	_CanasBus->setState(newState);

	uint8_t newMaster = msg->data.container.UCHAR4[1];
	ESP_LOGD(TAG, "New masternode=%d", newMaster);
	_CanasBus->setMasterNode(newMaster);

	myNodeId = _CanasBus->getNodeId();

	if (myNodeId == ICAN_Last_Node)
	{
		// nodeid not set yet so do not send response.
		ESP_LOGD(TAG, "no own node yet");
		return ICAN_INF_NO_NODEID_YET;
	}

	msg->node_id = myNodeId;
	//msg->service_code = 0;
	msg->data.type = CANAS_DATATYPE_UCHAR4;
	msg->data.container.UCHAR4[0] = myNodeId;
	msg->data.container.UCHAR4[1] = _myRequestPayload->hardware_revision;
	msg->data.container.UCHAR4[2] = _myRequestPayload->software_revision;
	msg->data.container.UCHAR4[3] = _CanasBus->getState();

	int ret = _CanasBus->ServiceSendResponse(msg, _myServiceId);
	_beaconConfirmed = true;

	if (ret != ICAN_ERR_OK)
	{
		ESP_LOGE(TAG, "srv ids: failed to respond:%d", ret);
	}
#endif
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
// request an identification service and chek if minimum hard/software versions are ok. Store all nodes that reply
//-------------------------------------------------------------------------------------------------------------------
int ICanService_beacon::Request(ulong timestamp)
{
#ifdef ICAN_MASTER
	CanasMessage msg;

	msg.message_code = 0;  // CANAS version == 0
	msg.data.type = CANAS_DATATYPE_UCHAR3;
	msg.service_code = ICAN_SRV_BEACON;
	msg.node_id = 0; // send to all nodes

	msg.data.container.UCHAR4[0] = _CanasBus->getState();
	msg.data.container.UCHAR4[1] = _CanasBus->getNodeId();
	msg.data.container.UCHAR4[2] = _myRequestPayload->software_revision;

	int ret = _CanasBus->ServiceSendRequest(&msg, _myServiceId);
	if (ret != ICAN_ERR_OK)
	{
		ESP_LOGE(TAG, "beacon srv ids: failed to respond:%d", ret);
		return ret;
	}
#endif
	return ICAN_ERR_OK;
}

//===================================================================================================================
// request to the main controler to stat providing a canareeospace data item on the bus
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
int ICanService_requestdata::Request(uint16_t dataId, int nodeId, int maxIntervalMs, bool newId)
{
	CanasMessage msg;

	ESP_LOGI(TAG, "CAN service request new data:%d interval=%d %s", dataId, maxIntervalMs, newId ? "NEW" : "");

	msg.node_id = nodeId;	// target node for request
	msg.data.type = CANAS_DATATYPE_USHORT2;
	msg.data.container.USHORT2[0] = dataId; // data item to transmit on the bus
	msg.data.container.UCHAR4[2] = _CanasBus->getNodeId(); // my own node id

	// set max value for interval
	if (maxIntervalMs > 2550) maxIntervalMs = 2550;

	msg.data.container.UCHAR4[3] = maxIntervalMs / 10; // max interval timing

	msg.service_code = _myServiceId;
	msg.message_code = (newId ? 1 : 0);

	_CanasBus->ServiceSendRequest(&msg, _myServiceId);

	return ICAN_ERR_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// TODO: support multiple request for same item
//-------------------------------------------------
int ICanService_requestdata::ProcessFrame(CanasMessage* msg)
{
	// TODO: register that server responded or that server is out
	if (msg->data.type != CANAS_DATATYPE_USHORT)
	{
		ESP_LOGE(TAG, "srv nss req: wrong data type %d", msg->data.type);
		return -ICAN_REQ_DATA_PARAM_ERROR;
	}

	uint16_t newCanId = (uint16_t)msg->data.container.USHORT;

	_CanasBus->ParamSubscribeCallback(newCanId, msg->message_code);
}
//--------------------------------------------------------------------------------------------------------------------
int ICanService_requestdata::Response(CanasMessage* msg)
{
	int result = ICAN_ERR_OK;
#ifdef ICAN_MASTER

	uint8_t serviceResult = ICAN_REQ_DATA_SUCCESS;
	int nodePointer = -1;
	uint8_t newNodeId;

	if (msg == NULL)
	{
		ESP_LOGE(TAG, "srv nss req: invalid state pointer");
		return -ICAN_ERR_ARGUMENT;
	}

	newNodeId = msg->data.container.UCHAR4[2];
	nodePointer = _findNodeInList(newNodeId);

	ESP_LOGI(TAG, "Request data request received for %d from %d", msg->data.container.USHORT2[0], newNodeId);

	if (msg->data.type != CANAS_DATATYPE_USHORT2)
	{
		ESP_LOGE(TAG, "srv nss req: wrong data type:%d", msg->data.type);
		serviceResult = ICAN_REQ_DATA_PARAM_ERROR;
	}
	else if (msg->service_code != _myServiceId)
	{
		ESP_LOGE(TAG, " srv nss req: wrong service code:%d ", msg->message_code);
		serviceResult = ICAN_REQ_DATA_BAD_MESSAGE_ID;
	}
	else if (nodePointer == -1)
	{
		// test if node already did register with master
		ESP_LOGW(TAG, "!! need to register on identification service first");
		serviceResult = ICAN_REQ_DATA_NOT_REGISTERED;
	}
	else if (_nodeRefs[nodePointer]->software_revision < ICAN_MIN_SOFTWARE_REVISION)
	{
		// test if software version is ok
		ESP_LOGW(TAG, "!! software version node is incorrect");
		serviceResult = ICAN_REQ_DATA_BAD_SOFTWARE_VERSION;
	}
	else
	{
		ESP_LOGD(TAG, "Done testing");

		uint16_t newCanId = msg->data.container.USHORT2[0];

		_canitemNode* tmpRef;
		_subscribedNode* newNode;

		//  check if we already provide this item ?
		int item = _findInCanIdList(newCanId);

		ESP_LOGD(TAG, "CAN service request for new item=%d msg=%d", newCanId, msg->message_code);

		if (msg->message_code == 0)
		{
			// request to remove an Item
			if (item == -1)
			{
				// not found so can not remove
				serviceResult = ICAN_REQ_DATA_NO_SUCH_ENTRY;
			}
			else
			{
				// found so remove item in list
				tmpRef = _canItemNodeRefs[item];
				if (_findNode(tmpRef, newNodeId, true) == 0)
					tmpRef->subscriptions--;
				else
					serviceResult = ICAN_REQ_DATA_NOT_SUBSCRIBED;

				if (tmpRef->subscriptions < 1)
				{
					_canItemNodeRefs.clear(item);
					result = _CanasBus->ParamUnadvertise(newCanId);
				}
			}
		}
		else
		{
			// request to add an item
			if (item != -1)
			{
				// found so update current item with this request
				ESP_LOGD(TAG, "Item found adding node");
				tmpRef = _canItemNodeRefs[item];

				if (_findNode(tmpRef, newNodeId))
				{
					serviceResult = ICAN_REQ_DATA_ENTRY_EXISTS;
				}
				else
				{
					ESP_LOGD(TAG, "Adding new node");
					tmpRef->subscriptions++;
					newNode = new _subscribedNode;
					newNode->nodeId = newNodeId;
					tmpRef->lastNode->next = newNode;
					newNode->prev = tmpRef->lastNode;
					tmpRef->lastNode = newNode;
					ESP_LOGD(TAG, "Adding new node subscriptions: %d", tmpRef->subscriptions);
				};
			}
			else
			{
				ESP_LOGD(TAG, "Item not found adding canref item");
				// test if XP element is available
				CanasXplaneTrans* cptab = Can2XPlane::fromCan2XplaneElement(newCanId);

				if (cptab->canasId == 0)
				{
					ESP_LOGI(TAG, "NO XPref found for canid");
					serviceResult = ICAN_REQ_DATA_NO_XREF;
				}
				else
				{
					ESP_LOGD(TAG, "Adding new canref item");
					// not found so add item in list
					tmpRef = new _canitemNode;
					tmpRef->canId = newCanId;
					newNode = new _subscribedNode;
					newNode->nodeId = newNodeId;
					tmpRef->firstNode = newNode;
					tmpRef->lastNode = newNode;
					tmpRef->subscriptions = 1;
					_canItemNodeRefs.push_back(tmpRef);

					ESP_LOGD(TAG, "CAN advertising new data element");

					//result = _CanasBus->ParamRegister(newCanId, false);
					result = _CanasBus->ParamAdvertise(newCanId, true, cptab->canIntervalMs);

					switch (result)
					{
					case ICAN_ERR_OK:
						break;
					case -ICAN_ERR_BAD_MESSAGE_ID:
						serviceResult = ICAN_REQ_DATA_INVALID_CANAS_ID;
						break;
					case -ICAN_ERR_ENTRY_EXISTS:
						serviceResult = ICAN_REQ_DATA_ENTRY_EXISTS;
						break;
					}
				}
			}
		}
		// return result of action
	};

	// some extra tests
	if (serviceResult == ICAN_REQ_DATA_SUCCESS)
	{
		// Check if XP interface is working
		if (!_CanasBus->isExternalBusRunning())
		{
			serviceResult = ICAN_REQ_DATA_XP_NOT_AVAILABLE;
		};
	};

	msg->data.type = CANAS_DATATYPE_USHORT;
	msg->message_code = serviceResult;

	_CanasBus->ServiceSendResponse(msg, _myServiceId);

#endif
	return result;
}
//===================================================================================================================
// Data accept request service class
//-------------------------------------------------------------------------------------------------------------------

int ICanService_acceptdata::Response(CanasMessage * msg)
{
#ifdef ICAN_MASTER
	// TODO: Code
	ESP_LOGI(TAG, "Receive data request for %d from %d", msg->data.container.USHORT2[0], msg->can_id);

#endif
	return ICAN_ERR_LOGIC;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

int ICanService_acceptdata::ProcessFrame(CanasMessage * msg)
{
	// TODO: Code
	return ICAN_ERR_LOGIC;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

int ICanService_acceptdata::Request(ulong timestamp)
{
	//TODO:Code
	return ICAN_ERR_LOGIC;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

int ICanService_acceptdata::Request(uint16_t dataId, int nodeId, int maxIntervalMs, bool newId)
{
	CanasMessage msg;

	ESP_LOGI(TAG, "CAN service request new accept data:%d interval=%d %s", dataId, maxIntervalMs, newId ? "NEW" : "");

	msg.node_id = nodeId;	// target node for request
	msg.data.type = CANAS_DATATYPE_USHORT2;
	msg.data.container.USHORT2[0] = dataId; // data item to transmit on the bus
	msg.data.container.UCHAR4[2] = _CanasBus->getNodeId(); // my own node id
	msg.data.container.UCHAR4[3] = maxIntervalMs; // max interval timing

	msg.service_code = _myServiceId;
	msg.message_code = (newId ? 1 : 0);

	_CanasBus->ServiceSendRequest(&msg, _myServiceId);

	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
// helper function to find a item i the list of  acepted items
//-------------------------------------------------------------------------------------------------------------------
int ICanService_acceptdata::_findInList(uint8_t toFind)
{
	int curRecord = -1;
	_canitemNode* tmpRef;

	ESP_LOGD(TAG, "find in list:%d", toFind);
	//DLVARPRINTLN(2,"items in list:",_canItemNodeRefs.size());

	for (int i = 0; i < _canItemNodeRefs.size(); i++)
	{
		ESP_LOGV(TAG, "check item:%d Compaire:%d with:%d", i, toFind, tmpRef->canId);

		tmpRef = _canItemNodeRefs[i];
		if (toFind == tmpRef->canId)
		{
			curRecord = i;
			break;
		}
	}

	ESP_LOGD(TAG, "find in list done:%d", curRecord);

	return curRecord;
}
//===================================================================================================================
// get a node id from master node service class
//-------------------------------------------------------------------------------------------------------------------
ICanService_getNodeId::ICanService_getNodeId(ICanBase * CanAsBus) : ICanService(CanAsBus, ICAN_SRV_GETNODEID)
{
	uint8_t macId[6];

	//_mySerial = ESP.getEfuseMac();
	if (esp_efuse_mac_get_default(macId) == ESP_OK)
	{
		_mySerial = 0;
		for (int i = 0; i < 6; i++)
		{
			ESP_LOGD(TAG, "MAC[%d][%d][%x]", i, macId[i], macId[i]);
			_mySerial += macId[i] * 256 ^ (i + 1);
		}
	}
	else
		ESP_LOGE(TAG, "Error getting MacId");

	ESP_LOGD(TAG, "Serial=%x", (unsigned long)_mySerial);
}

//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanService_getNodeId::Response(CanasMessage * msg)
{
#ifdef ICAN_MASTER
	// check if it is request or reply
	_node *curNode, *lastNode = NULL, *newNode = NULL;
	uint32_t newSerial;

	if (msg == NULL)
	{
		ESP_LOGE(TAG, "srv ids response: bad msg param");
		return -ICAN_ERR_ARGUMENT;
	}

	if (msg->data.type != CANAS_DATATYPE_ULONG)
	{
		return -ICAN_ERR_BAD_DATA_TYPE;
	}

	// find in list
	curNode = _allNodes;
	newSerial = msg->data.container.ULONG;

	ESP_LOGI(TAG, "New Node id request received for serial %x", newSerial);

	while ((curNode != NULL) && (curNode->nodeSerial != newSerial))
	{
		ESP_LOGV(TAG, "INLIST=%x", curNode->nodeSerial);

		lastNode = curNode;
		curNode = curNode->nextNode;
	};

	if (curNode == NULL)
	{
		newNode = new _node;
		newNode->nodeSerial = newSerial;
		newNode->nodeId = _lastNodeId++;
		newNode->lastNode = curNode;
		ESP_LOGI(TAG, "Added as node %d", newNode->nodeId);

		if (_allNodes != NULL)
		{
			ESP_LOGD(TAG, "Adding at lastnode");
			lastNode->nextNode = newNode;
		}
		else
			_allNodes = newNode;
	}
	else
	{
		// found so return nodeid;
		newNode = curNode;
	};

	ESP_LOGD(TAG, "Sending newNode =%d", newNode->nodeId);

	msg->message_code = newNode->nodeId;

	int ret = _CanasBus->ServiceSendResponse(msg, _myServiceId);
	if (ret != 0)
	{
		ESP_LOGI(TAG, "!!srv ids: failed to respond:%d", ret);
	}
#endif
	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanService_getNodeId::ProcessFrame(CanasMessage * msg)
{
	// TODO: Code

	if (msg == NULL)
	{
		ESP_LOGE(TAG, "srv ids response: bad msg param");
		return -ICAN_ERR_ARGUMENT;
	}

	if (msg->data.type != CANAS_DATATYPE_ULONG)
	{
		ESP_LOGE(TAG, "srv ids response: bad data type");
		return -ICAN_ERR_BAD_DATA_TYPE;
	}

	ESP_LOGD(TAG, "in msg:%u :in module:%u", msg->data.container.ULONG, (ulong)_mySerial);
	if (msg->data.container.ULONG == (ulong)_mySerial)
	{
		// this is reply to my request so set Node Id
		ESP_LOGD(TAG, "setting node id");
		_CanasBus->setNodeId(msg->message_code);
	}

	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

int ICanService_getNodeId::Request(uint8_t masterNode, bool newId)
{
	CanasMessage msg;

	ESP_LOGD(TAG, "GetNodeId request to%d", masterNode);

	msg.node_id = masterNode;	// target node for request
	msg.data.type = CANAS_DATATYPE_ULONG;

	msg.data.container.ULONG = _mySerial;

	msg.service_code = _myServiceId;
	msg.message_code = (newId ? 1 : 0);;

	_CanasBus->ServiceSendRequest(&msg, _myServiceId);

	return ICAN_ERR_OK;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------