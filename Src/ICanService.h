//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// ICansERVICE.h
// Can areo like services.
//
// VERSION HISTORY:
//
//==================================================================================================
// A light implementation of the CAN Aerospace protocol to manage simulated instruments.
//	This code is in the public domain.
//
// Thanks to mjs513/CANaerospace (Pavel Kirienko, 2013 (pavel.kirienko@gmail.com))
//-------------------------------------------------------------------------------------------------------------------
//#pragma once

#ifndef _ICAN_SERVICE_H_
#define _ICAN_SERVICE_H_

#include "XPCAN_messages.h"
#include "ICanBase.h"

typedef enum {
	ICAN_SRV_BEACON = 0,
	ICAN_SRV_TIMESTAMP = 1,			// not implemented yet
	ICAN_SRV_GETNODEID = 11,
	ICAN_SRV_REQUEST_CANDATA = 100,
	ICAN_SRV_ACCEPT_CANDATA = 101
} ICanSrvCodes;

typedef enum {
	ICAN_REQ_DATA_SUCCESS = 0,
	ICAN_REQ_DATA_XP_NOT_AVAILABLE,
	ICAN_REQ_DATA_NOT_REGISTERED,
	ICAN_REQ_DATA_INVALID_CANAS_ID,
	ICAN_REQ_DATA_NO_XREF,
	ICAN_REQ_DATA_PARAM_ERROR,
	ICAN_REQ_DATA_BAD_SOFTWARE_VERSION,
	ICAN_REQ_DATA_BAD_MESSAGE_ID,
	ICAN_REQ_DATA_NO_SUCH_ENTRY,
	ICAN_REQ_DATA_ENTRY_EXISTS,
	ICAN_REQ_DATA_NOT_SUBSCRIBED,
} ICanDataRequestSrvCodes;

constexpr uint8_t ICAN_First_Node = 2;
constexpr uint8_t ICAN_Last_Node = 255;

/**
 * Data to be transferred by the Beacon service
 */
typedef struct {
	uint8_t node_state;
	uint8_t hardware_revision;
	uint8_t software_revision;
	uint8_t notused;
} ICanBeaconRequestPayload;

typedef struct {
	uint8_t master_state;
	uint8_t master_node;
	uint8_t software_revision;
	uint8_t notused;
} ICanasBeaconReplyPayload;

typedef struct {
	uint16_t	dataId;
	uint16_t	interval;
} ICanSrvDataItemRequestPayload;

typedef struct {
	uint8_t		status;
	uint8_t		notused;
	uint16_t	notused2;
} ICanSrvDataItemReplyPayload;

typedef struct {
	ulong		indentifier;
} ICanSrvGetNodeidPayload;

class ICanService {
public:
	ICanService(ICanBase *CanAsBus, ICanSrvCodes serviceID);

	~ICanService() {/*_CanasBus->ServiceUnregister(_myServiceId);*/ };

	virtual int Response(CanasMessage* msg) = 0;
	//	virtual int Request(CanasMessage* msg) = 0; // is implemented in child classes
	virtual int ProcessFrame(CanasMessage* msg) = 0;
	virtual int Request(ulong timestamp) = 0;
	uint8_t serviceId() { return _myServiceId; };
	void checkNodes(long timestamp);

protected:
	// TODO : Clean up with extra subclass
	struct _subscribedNode {
		uint8_t		nodeId;
		uint8_t		intervalMs = 100;
		_subscribedNode* next = NULL;
		_subscribedNode* prev = NULL;
	};

	struct _canitemNode {
		uint16_t			canId = 0;
		int					subscriptions = 0;
		ulong				timestamp = 0;
		_subscribedNode*	firstNode = NULL;
		_subscribedNode*	lastNode = NULL;
	};
	static QList < _canitemNode*> _canItemNodeRefs;

	int _findNode(_canitemNode* myStruct, uint8_t nodeId, bool doDelete = false);
	int _findInCanIdList(uint16_t toFind);
	int _serviceChannelFromMessageID(canbusId_t msg_id, bool * pisrequest);

	typedef struct {
		uint8_t		nodeId;
		uint8_t		hardware_revision;
		uint8_t		software_revision;
		uint8_t		id_distribution;
		uint8_t		nodeState;
		unsigned long		timestamp = 0;
	} _ICanNodeInfo;

	static QList < _ICanNodeInfo*> _nodeRefs;
	static int _findNodeInList(uint8_t toFind);

	uint8_t			_myServiceId = -1;
	ICanBase*		_CanasBus = NULL;
	bool			_canAnswer;
	bool			_canrequest;
	ulong			_lastCleanupTs = 0;
};

//---------------------------------------------------------------------------------------------------------------------
// Beacon service
//---------------------------------------------------------------------------------------------------------------------
//|Message	| datafield		|		| service		|		| service	|
//|DataByte | description	|		| request		|		| response	|
//| ----	| -- -			|-------| -- -			| -- -	| ----		|
//| 0		| Node ID		| UCHAR | 0				|		| <node_id>	|
//| 1		| Data Type		| UCHAR | UCHAR3		|		| UCHAR4	|
//| 2		| Service Code	| UCHAR | 0				|		| 0
//| 3		| Message Code	| UCHAR | 0				|		| 0
//| 4		| Data[0]		| UCHAR | MasterState	| UCHAR | NodeId
//| 5		| Data[1]		| UCHAR | MasterNodeID	| UCHAR | HW Version
//| 6		| Data[2]		| UCHAR | SW Version	| UCHAR | SW version
//| 7		| Data			|		|				| UCHAR | Node State
//---------------------------------------------------------------------------------------------------------------------
class ICanService_beacon : public ICanService {
public:
	ICanService_beacon(ICanBase* CanAsBus, ICanBeaconRequestPayload* payload);
	~ICanService_beacon();

	int Response(CanasMessage* msg);
	int ProcessFrame(CanasMessage* msg);
	int Request(ulong timestamp);
	bool isBeaconConfirmed() { return _beaconConfirmed; };

private:

	ICanBeaconRequestPayload* _myRequestPayload = NULL;
	ICanasBeaconReplyPayload*  _myReplyPayload = NULL;
	bool _beaconConfirmed = false;
};

//---------------------------------------------------------------------------------------------------------------------
// Get Node id service
//---------------------------------------------------------------------------------------------------------------------
//|Message	| datafield		|		| service		|		| service
//|DataByte	| description	|		| request		|		| response
//|---------|---------------|-------|---------------|-------|-----------|
//| 0		| Node ID		| UCHAR |Master Node ID |		| 255 		|
//| 1		| Data Type		| UCHAR | ULONG			|		| ULONG		|
//| 2		| Service Code	| UCHAR | 11			|		| 11
//| 3		| Message Code	| UCHAR | 0	= release,	|		| New NodeID
//|			|				|		| 1= request	|		|
//| 4-7		| Data			| ULONG | uniqueID		| ULONG | uniqueID
//---------------------------------------------------------------------------------------------------------------------
class ICanService_getNodeId : public ICanService {
public:
	ICanService_getNodeId(ICanBase* CanAsBus);
	~ICanService_getNodeId() {};

	int Response(CanasMessage* msg);
	int ProcessFrame(CanasMessage* msg);
	int Request(ulong timestamp) { return -1; };
	int Request(uint8_t masterNode, bool newId);

private:
	struct _node {
		uint8_t nodeId;
		ulong nodeSerial;
		_node* nextNode = NULL;
		_node* lastNode = NULL;
	};

	_node*	_allNodes = NULL;
	_node*	_lastNode = NULL;
	uint64_t _mySerial = 0;
	uint8_t _lastNodeId = 2;
};

//---------------------------------------------------------------------------------------------------------------------
// service get data class Class definition
//---------------------------------------------------------------------------------------------------------------------
//Message	| datafield		|		| service			|		| service
//DataByte	| description	|		| request			|		| response
//--------------------------------------------------------------
//0			| Node - ID		| UCHAR | <node - ID>		|		| < node - ID>
//1			| Data Type		| UCHAR | USHORT2			|		| USHORT
//2			| Service code	| UCHAR | 100				|		| 100
//3			| Message Code	| UCHAR | 0 = stop; 1 = new |		| !0 = error
//4 - 5		| Data			|USHORT | Canas ID			|USHORT	| Canas ID
//6			| Data			| UCHAR | NodeId		 	|		| n.a.
//7			| Data			| UCHAR	| n.a.				|		| n.a.
//--------------------------------------------------------------------------------------------------------------------
class ICanService_requestdata : public ICanService {
public:
	ICanService_requestdata(ICanBase* CanAsBus) : ICanService(CanAsBus, ICAN_SRV_REQUEST_CANDATA) {};
	~ICanService_requestdata() {};

	int Response(CanasMessage* msg);
	int ProcessFrame(CanasMessage* msg);
	int Request(ulong timestamp) { return -ICAN_ERR_LOGIC; };
	int Request(uint16_t dataId, int nodeId, int maxIntervalMs = CANAS_DEFAULT_SERVICE_POLL_INTERVAL_MSEC, bool newId = true);
private:
};

//---------------------------------------------------------------------------------------------------------------------
// service accept data Class definition
//---------------------------------------------------------------------------------------------------------------------
//Message	| datafield		|		| service			|| service
//DataByte	| description	|		| request			|| response
//--------------------------------------------------------------
//0			| Node - ID		| UCHAR | <node - ID>		|| < node - ID>
//1			| Data Type		| UCHAR | USHORT2			|| NODATA
//2			| Service code	| UCHAR | 101				|| 101
//3			| Message Code	| UCHAR | 0 = stop; 1 = new || !0 = error
//4 - 5		| Data			|USHORT | Canas ID			|| n.a.
//6			| Data			| UCHAR | NodeId			|| n.a.
//7			| Data			| UCHAR	| Send Interval		|| n.a.
//--------------------------------------------------------------------------------------------------------------------
class ICanService_acceptdata : public ICanService {
public:
	ICanService_acceptdata(ICanBase* CanAsBus) : ICanService(CanAsBus, ICAN_SRV_ACCEPT_CANDATA) {};
	~ICanService_acceptdata() {};

	int Response(CanasMessage* msg);
	int ProcessFrame(CanasMessage* msg);
	int Request(ulong timestamp);
	int Request(uint16_t dataId, int nodeId, int maxIntervalMs = CANAS_DEFAULT_SERVICE_POLL_INTERVAL_MSEC, bool newId = true);
private:

	struct _dataAccept {
		uint8_t		canId = 0;
		int			subscriptions = 0;
		_subscribedNode* firstNode = NULL;
		_subscribedNode* lastNode = NULL;
	};
	QList < _dataAccept*> _dataAcceptRefs;
	_subscribedNode* _findNode(_subscribedNode* myStruct, uint8_t nodeId, bool doDelete = false);

	int _findInList(uint8_t toFind);
};

#endif //_ICAN_SERVICE_H_
