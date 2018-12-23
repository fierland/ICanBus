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
#include "ICanService.h"
#include "Can2XPlane.h"
#include "ican_debug.h"

QList <ICanService::_ICanNodeInfo*> ICanService::_nodeRefs;

#define RANGEINCLUSIVE(x, min, max) ((x) >= (min) && (x) <= (max))
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanService::_serviceChannelFromMessageID(canbusId_t msg_id, bool* pisrequest)
{
	DPRINTINFO("START");
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

	DPRINTINFO("STOP");

	return -ICAN_ERR_BAD_MESSAGE_ID;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
ICanService::ICanService(ICanBase * CanAsBus, ICanSrvCodes serviceID)
{
	DPRINTINFO("START");
	/*
	if (CanAsBus == NULL)
	{
		DPRINTLN("CANAS_service init: bad payload param");
		return;
	};
	*/
	DPRINT("Adding service :"); DPRINTLN(serviceID);
	_CanasBus = CanAsBus;
	_myServiceId = serviceID;

	DPRINTINFO("STOP");
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
ICanService::~ICanService()
{
	DPRINTINFO("START");
	//_CanasBus->ServiceUnregister(_myServiceId);
	DPRINTINFO("STOP");
}
uint8_t ICanService::serviceId()
{
	return _myServiceId;
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int ICanService::_findNodeInList(uint8_t toFind)
{
	int curRecord = -1;
	_ICanNodeInfo* tmpRef;

#if DEBUG_LEVEL > 5
	DPRINTINFO("START");
	DPRINT("find in list:"); DPRINTLN(toFind); DPRINT("items in list:"); DPRINTLN(_nodeRefs.size());
#endif

	for (int i = 0; i < _nodeRefs.size(); i++)
	{
		tmpRef = _nodeRefs[i];
#if DEBUG_LEVEL > 5
		DPRINT("check item:"); DPRINTLN(i);
		DPRINT("Compaire:"); DPRINT(toFind); DPRINT(":with:"); DPRINTLN(tmpRef->nodeId);
#endif

		if (toFind == tmpRef->nodeId)
		{
			curRecord = i;
			break;
		}
	}
#if DEBUG_LEVEL > 5
	DPRINT("find in list done:"); DPRINTLN(curRecord);
	DPRINTINFO("STOP");
#endif
	return curRecord;

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
int ICanService::_findNode(_canitemNode* myStruct, uint8_t nodeId, bool doDelete)
{
	int i = 0;
	_subscribedNode* curNode;
	_subscribedNode* prevNode;
	int result = 0;

#if DEBUG_LEVEL > 1
	DPRINTINFO("START");
	DPRINT("find node:"); DPRINTLN(nodeId);
#endif
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
				DPRINTLN("Deleting node");
				prevNode->next = curNode->next;
				delete curNode;
			}
			return 0;
		}
		DPRINTLN("Adding node");
		prevNode = curNode;
		curNode = curNode->next;
	}

	result = -ICAN_ERR_BAD_NODE_ID;

#if DEBUG_LEVEL > 1
	DPRINT("find node done:"); DPRINTLN(result);
	DPRINTINFO("STOP");
#endif
	return result;
}
//===================================================================================================================
// Beacon service class
//-------------------------------------------------------------------------------------------------------------------

ICanService_beacon::ICanService_beacon(ICanBase * CanAsBus, ICanBeaconRequestPayload * payload) : ICanService(CanAsBus, ICAN_SRV_BEACON)
{
	DPRINTINFO("START");
	if (payload == NULL)
	{
		DPRINTLN("srv ids init: bad payload param");
		return;
	}
	_myRequestPayload = payload;
	DPRINTINFO("STOP");
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
ICanService_beacon::~ICanService_beacon()
{
	DPRINTINFO("START");
	delete _myRequestPayload;
	DPRINTINFO("STOP");
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanService_beacon::ProcessFrame(CanasMessage * msg)
{
	DPRINTINFO("START");
	_ICanNodeInfo* newNode;
	int curRecord;
	uint8_t newNodeId;

	if (msg->data.type != CANAS_DATATYPE_UCHAR4)
	{
		DPRINTLN("!!! Bad data typr)");
		return -ICAN_ERR_BAD_DATA_TYPE;
	}

	newNodeId = (uint8_t)msg->data.container.UCHAR4[0];
	curRecord = _findNodeInList(newNodeId);

	if (curRecord != -1)
	{
#if DEBUG_LEVEL > 2
		Serial.print("CAN beacon Found Node:"); Serial.println(newNodeId);
#endif

		newNode = _nodeRefs[curRecord];
	}
	else
	{
#if DEBUG_LEVEL > 0
		Serial.print("CAN beacon Adding node:"); Serial.println(newNodeId);
#endif
		DPRINT("Added Node for:"); DPRINT(msg->node_id);	DPRINT(" HWid="); DPRINT((uint8_t)msg->data.container.UCHAR4[0], HEX);
		DPRINT(" SWid="); DPRINT((uint8_t)msg->data.container.UCHAR4[1], HEX);

		newNode = new _ICanNodeInfo;
		newNode->nodeId = newNodeId;
		newNode->hardware_revision = msg->data.container.UCHAR4[1];
		newNode->software_revision = msg->data.container.UCHAR4[2];
		_nodeRefs.push_back(newNode);
	};
	// TODO: neat timestamp
	newNode->timestamp = millis();
	newNode->nodeState = msg->data.container.UCHAR4[3];

	DPRINTINFO("STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// send response to a identification service request`
//-------------------------------------------------------------------------------------------------------------------
int ICanService_beacon::Response(CanasMessage * msg)
{
	// check if it is request or reply
	uint8_t myNodeId;

	DLPRINTINFO(3, "START");
	if (msg == NULL)
	{
		DLPRINTLN(1, "srv ids response: bad msg param");
		return -ICAN_ERR_ARGUMENT;
	}

	// check if master node is correct
	//msg->node_id = msg->data.container.UCHAR4[1];
	DLPRINT(2, "New state="); DLPRINTLN(0, msg->data.container.UCHAR4[0], BIN);

	uint8_t newState = msg->data.container.UCHAR4[0];
	_CanasBus->setState(newState);

	uint8_t newMaster = msg->data.container.UCHAR4[1];
	_CanasBus->setMasterNode(newMaster);

	myNodeId = _CanasBus->getNodeId();
	if (myNodeId == ICAN_Last_Node)
	{
		// nodeid not set yet so do not send response.
		DLPRINTLN(1, "no own node yet");
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

	if (ret != 0)
	{
		DLPRINT(1, "srv ids: failed to respond:");
		DPRINTLN(ret);
	}
	DLPRINTINFO(3, "STOP");

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// request an identification service and chek if minimum hard/software versions are ok. Store all nodes that reply
//-------------------------------------------------------------------------------------------------------------------
int ICanService_beacon::Request(ulong timestamp)
{
	DLPRINTINFO(4, "START");
	CanasMessage msg;

	msg.message_code = 0;  // CANAS version == 0
	msg.data.type = CANAS_DATATYPE_UCHAR3;
	msg.service_code = ICAN_SRV_BEACON;
	msg.node_id = 0; // send to all nodes

	msg.data.container.UCHAR4[0] = _CanasBus->getState();
	msg.data.container.UCHAR4[1] = _CanasBus->getNodeId();
	msg.data.container.UCHAR4[2] = _myRequestPayload->software_revision;

	int ret = _CanasBus->ServiceSendRequest(&msg, _myServiceId);
	if (ret != 0)
	{
		DPRINT("srv ids: failed to respond:");
		DPRINTLN(ret);
		return ret;
	}

	// cleanup for stations not responding  ?

	if ((_lastCleanupTs + CANAS_NODE_CLEANUP_MSEC) < timestamp)
	{
		for (int i = 0; i < _nodeRefs.length(); i++)
		{
			if ((_nodeRefs[i]->timestamp + CANAS_NODE_TIMEOUT_MSEC) < timestamp)
			{
				// node seems dead
				DPRINT("Killing node:"); DPRINTLN(_nodeRefs[i].nodeId);
				// clenup nodeinfo
				// TODO:CLEANUP CODE
			}
		}
	}

	DLPRINTINFO(4, "STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
bool ICanService_beacon::isBeaconConfirmed()
{
	return _beaconConfirmed;
}

//===================================================================================================================
// request to the main controler to stat providing a canareeospace data item on the bus
//-------------------------------------------------------------------------------------------------------------------

ICanService_requestdata::ICanService_requestdata(ICanBase* CanAsBus) : ICanService(CanAsBus, ICAN_SRV_REQUEST_CANDATA)
{
	DPRINTINFO("START");

	DPRINTINFO("STOP");
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------

ICanService_requestdata::~ICanService_requestdata()
{
	DPRINTINFO("START");

	DPRINTINFO("STOP");
}

//-------------------------------------------------------------------------------------------------------------------
int ICanService_requestdata::Request(uint16_t dataId, int nodeId, int maxIntervalMs, bool newId)
{
	CanasMessage msg;
	DLPRINTINFO(2, "START");

#if DEBUG_LEVEL > 0
	Serial.print("CAN service request new data:"); Serial.print(dataId); Serial.print(" interval="); Serial.print(maxIntervalMs); Serial.print(" new?="); Serial.println(newId);
#endif

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

	DLPRINTINFO(2, "STOP");

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// helper function to find a item i the list of  subscribed items
//-------------------------------------------------------------------------------------------------------------------
int ICanService_requestdata::_findInCanIdList(uint16_t toFind)
{
	int curRecord = -1;
	_canitemNode* tmpRef;

	DLPRINTINFO(2, "START");

	DLVARPRINT(5, "CAN find canid in list:", toFind); DLVARPRINTLN(5, "items in list:", _canItemNodeRefs.size());

	//DPRINTLN(_dataReqRefs.size());

	for (int i = 0; i < _canItemNodeRefs.size(); i++)
	{
		DLVARPRINT(3, "check item:", i);
		DLVARPRINT(3, "Compaire:", toFind); DLVARPRINTLN(3, "with:", tmpRef->canId);

		tmpRef = _canItemNodeRefs[i];
		if (toFind == tmpRef->canId)
		{
			curRecord = i;
			break;
		}
	}

	DLVARPRINTLN(5, "find canid done:", curRecord);
	DLPRINTINFO(2, "STOP");

	return curRecord;
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
int ICanService_requestdata::Request(ulong timestamp)
{
	// dummy function not used;
	return -1;
}

//-------------------------------------------------------------------------------------------------------------------
// TODO: support multiple request for same item
//-------------------------------------------------
int ICanService_requestdata::ProcessFrame(CanasMessage* msg)
{
	DLPRINTINFO(2, "START");
	// TODO: register that server responded or that server is out
	if (msg->data.type != CANAS_DATATYPE_USHORT)
	{
		DLPRINT(0, "srv nss req: wrong data type ");
		DLPRINTLN(0, msg->data.type);
		return -ICAN_REQ_DATA_PARAM_ERROR;
	}

	uint16_t newCanId = (uint16_t)msg->data.container.USHORT;

	_CanasBus->ParamSubscribeCallback(newCanId, msg->message_code);

	DLPRINTINFO(2, "STOP");
}
//--------------------------------------------------------------------------------------------------------------------
int ICanService_requestdata::Response(CanasMessage* msg)
{
	int result = 0;
	DPRINTINFO("START");
	uint8_t serviceResult = ICAN_REQ_DATA_SUCCESS;
	int nodePointer = -1;
	uint8_t newNodeId;

	if (msg == NULL)
	{
		DLPRINTLN(0, "srv nss req: invalid state pointer");
		return -ICAN_ERR_ARGUMENT;
	}

	newNodeId = msg->data.container.UCHAR4[2];
	nodePointer = _findNodeInList(newNodeId);

	if (msg->data.type != CANAS_DATATYPE_USHORT2)
	{
		DLPRINT(0, "srv nss req: wrong data type ");
		DLPRINTLN(0, msg->data.type);
		serviceResult = ICAN_REQ_DATA_PARAM_ERROR;
	}
	else if (msg->service_code != _myServiceId)
	{
		DLPRINT(0, " srv nss req: wrong service code ");
		DLPRINTLN(0, msg->message_code);
		serviceResult = ICAN_REQ_DATA_BAD_MESSAGE_ID;
	}
	else if (nodePointer == -1)
	{
		// test if node already did register with master
		DLPRINTLN(0, "!! need to register on indetification service first");
		serviceResult = ICAN_REQ_DATA_NOT_REGISTERED;
	}
	else if (_nodeRefs[nodePointer]->software_revision < ICAN_MIN_SOFTWARE_REVISION)
	{
		// test if software version is ok
		DLPRINTLN(0, "!! software version node is incorrect");
		serviceResult = ICAN_REQ_DATA_BAD_SOFTWARE_VERSION;
	}
	else
	{
		DLPRINTLN(4, "Done testing");

		uint16_t newCanId = msg->data.container.USHORT2[0];

		_canitemNode* tmpRef;
		_subscribedNode* newNode;

		//  check if we already provide this item ?
		int item = _findInCanIdList(newCanId);

		DLPRINT(1, "CAN serive request for new item="); DLPRINT(1, newCanId); DLPRINT(1, " msg="); DLPRINTLN(1, msg->message_code);

		if (msg->message_code == 0)
		{
			// request to remove an Item
			if (item == 0)
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
				DLPRINTLN(1, "Item found adding node");
				tmpRef = _canItemNodeRefs[item];

				if (_findNode(tmpRef, newNodeId))
				{
					serviceResult = ICAN_REQ_DATA_ENTRY_EXISTS;
				}
				else
				{
					DLPRINTLN(1, "Adding new node");
					tmpRef->subscriptions++;
					newNode = new _subscribedNode;
					newNode->nodeId = newNodeId;
					tmpRef->lastNode->next = newNode;
					tmpRef->lastNode = newNode;
				};
			}
			else
			{
				DLPRINTLN(1, "Item not found adding canref item");
				// test if XP element is available
				CanasXplaneTrans* cptab = Can2XPlane::fromCan2XplaneElement(newCanId);

				if (cptab->canasId == 0)
				{
					DLPRINTLN(0, "NO XPref found for canid");
					serviceResult = ICAN_REQ_DATA_NO_XREF;
				}
				else
				{
					DLPRINTLN(1, "Adding new canref item");
					// not found so add item in list
					tmpRef = new _canitemNode;
					tmpRef->canId = newCanId;
					newNode = new _subscribedNode;
					newNode->nodeId = newNodeId;
					tmpRef->firstNode = newNode;
					tmpRef->lastNode = newNode;
					tmpRef->subscriptions = 1;
					_canItemNodeRefs.push_back(tmpRef);

					DLPRINTLN(1, "CAN advertising new data element");

					//result = _CanasBus->ParamRegister(newCanId, false);
					result = _CanasBus->ParamAdvertise(newCanId, true, cptab->canIntervalMs);

					switch (result)
					{
					case 0:
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

	DPRINTINFO("STOP");
	return result;
}
//===================================================================================================================
// Data accept request service class
//-------------------------------------------------------------------------------------------------------------------
ICanService_acceptdata::ICanService_acceptdata(ICanBase * CanAsBus) : ICanService(CanAsBus, ICAN_SRV_ACCEPT_CANDATA)
{}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
ICanService_acceptdata::~ICanService_acceptdata()
{}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

int ICanService_acceptdata::Response(CanasMessage * msg)
{
	// TODO :Code
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

int ICanService_acceptdata::ProcessFrame(CanasMessage * msg)
{
	// TODO :Code
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

int ICanService_acceptdata::Request(ulong timestamp)
{
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------

int ICanService_acceptdata::Request(uint16_t dataId, int nodeId, int maxIntervalMs, bool newId)
{
	CanasMessage msg;
	DPRINTINFO("START");

	msg.node_id = nodeId;	// target node for request
	msg.data.type = CANAS_DATATYPE_USHORT2;
	msg.data.container.USHORT2[0] = dataId; // data item to transmit on the bus
	msg.data.container.UCHAR4[2] = _CanasBus->getNodeId(); // my own node id
	msg.data.container.UCHAR4[3] = maxIntervalMs; // max interval timing

	msg.service_code = _myServiceId;
	msg.message_code = (newId ? 1 : 0);

	_CanasBus->ServiceSendRequest(&msg, _myServiceId);

	DPRINTINFO("STOP");

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// helper function to find a item i the list of  subscribed items
//-------------------------------------------------------------------------------------------------------------------
int ICanService_acceptdata::_findInList(uint8_t toFind)
{
	int curRecord = -1;
	_canitemNode* tmpRef;

	DLPRINTINFO(2, "START");

	DLVARPRINT(2, "find in list:", toFind);
	//DLVARPRINTLN(2,"items in list:",_canItemNodeRefs.size());

	for (int i = 0; i < _canItemNodeRefs.size(); i++)
	{
		DLVARPRINTLN(2, "check item:", i); DLVARPRINT(2, "Compaire:", toFind); DLVARPRINTLN(2, "with:", tmpRef->canId);

		tmpRef = _canItemNodeRefs[i];
		if (toFind == tmpRef->canId)
		{
			curRecord = i;
			break;
		}
	}

	DLVARPRINTLN(2, "find in list done:", curRecord);
	DLPRINTINFO(2, "STOP");

	return curRecord;
}
//===================================================================================================================
// get a node id from master node service class
//-------------------------------------------------------------------------------------------------------------------
ICanService_getNodeId::ICanService_getNodeId(ICanBase * CanAsBus) : ICanService(CanAsBus, ICAN_SRV_GETNODEID)
{
	uint64_t macId;
	DLPRINTINFO(1, "START");

	_mySerial = ESP.getEfuseMac();

	DLVARPRINT(2, "Serial=", (unsigned long)_mySerial, HEX);

	DLPRINTINFO(1, "STOP");
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
ICanService_getNodeId::~ICanService_getNodeId()
{
	// TODO: neat cleanup
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanService_getNodeId::Response(CanasMessage * msg)
{
	// check if it is request or reply
	_node *curNode, *lastNode = NULL, *newNode = NULL;
	uint32_t newSerial;

	DLPRINTINFO(3, "START");
	if (msg == NULL)
	{
		DLPRINTLN(0, "srv ids response: bad msg param");
		return -ICAN_ERR_ARGUMENT;
	}

	if (msg->data.type != CANAS_DATATYPE_ULONG)
	{
		return -ICAN_ERR_BAD_DATA_TYPE;
	}

	// find in list
	curNode = _allNodes;
	newSerial = msg->data.container.ULONG;
	DLVARPRINTLN(2, "Serial=", newSerial);

	while ((curNode != NULL) && (curNode->nodeSerial != newSerial))
	{
		DLVARPRINTLN(2, "INLIST=", curNode->nodeSerial);

		lastNode = curNode;
		curNode = curNode->nextNode;
	};

	if (curNode == NULL)
	{
		DLPRINT(2, "not found so make new one");
		newNode = new _node;
		newNode->nodeSerial = newSerial;
		DLVARPRINTLN(2, "New node =", _lastNodeId);
		newNode->nodeId = _lastNodeId++;
		newNode->lastNode = curNode;

		DLVARPRINT(1, "CAN Add Node:", newNode->nodeId);

		if (_allNodes != NULL)
		{
			DLPRINTLN(2, "Adding at lastnode");
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

	DLVARPRINTLN(1, "Sending newNode =", newNode->nodeId);

	msg->message_code = newNode->nodeId;

	int ret = _CanasBus->ServiceSendResponse(msg, _myServiceId);
	if (ret != 0)
	{
		DLVARPRINTLN(0, "!!srv ids: failed to respond:", ret);
	}
	DLPRINTINFO(3, "STOP");

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanService_getNodeId::ProcessFrame(CanasMessage * msg)
{
	// TODO: Code
	DLPRINTINFO(3, "START");
	if (msg == NULL)
	{
		DLPRINTLN(0, "srv ids response: bad msg param");
		return -ICAN_ERR_ARGUMENT;
	}

	if (msg->data.type != CANAS_DATATYPE_ULONG)
	{
		DLPRINTLN(0, "srv ids response: bad data type");
		return -ICAN_ERR_BAD_DATA_TYPE;
	}

	DLVARPRINT(2, "in msg:", msg->data.container.ULONG); DLVARPRINTLN(2, ":in module:", (ulong)_mySerial);
	if (msg->data.container.ULONG == (ulong)_mySerial)
	{
		// this is reply to my request so set Node Id
		DLPRINTLN(2, "setting node id");
		_CanasBus->setNodeId(msg->message_code);
	}

	DLPRINTINFO(3, "STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------
int ICanService_getNodeId::Request(ulong timestamp)
{
	return -1;
}

int ICanService_getNodeId::Request(uint8_t masterNode, boolean newId)
{
	CanasMessage msg;
	DLPRINTINFO(3, "START");

	msg.node_id = masterNode;	// target node for request
	msg.data.type = CANAS_DATATYPE_ULONG;

	msg.data.container.ULONG = _mySerial;

	msg.service_code = _myServiceId;
	msg.message_code = (newId ? 1 : 0);;

	_CanasBus->ServiceSendRequest(&msg, _myServiceId);

	DLPRINTINFO(3, "STOP");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------