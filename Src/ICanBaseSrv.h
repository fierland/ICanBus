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
#ifndef _ICAN_BASESRV_H_
#define _ICAN_BASESRV_H_

#include "ICanBase.h"
#include "ICanService.h"

class ICanBaseSrv :
	public ICanBase {
public:
	ICanBaseSrv(nodeId_t nodeId, uint8_t hdwId, uint8_t swId);

	~ICanBaseSrv();

	int ParamUnsubscribe(canbusId_t msg_id);

	int ServiceRegister(ICanService* newService);
	int ServiceUnregister(service_channel_t service_code);
	int ServiceSubscribe(service_channel_t service_code, bool receive = true);

	int ServiceAdvertise(service_channel_t service_code, long interval = CANAS_DEFAULT_SERVICE_ADVERTISE_TIMEOUT_MSEC);
	int ServicePublish(service_channel_t service_code);
	virtual int ServicePublish(int currentRecord);

	int ServicePublish(int currentRecord, long timestamp);

	// generic callback for can filter
	static void CallBackData(CAN_FRAME* frame);
	static void CallBackService(CAN_FRAME* frame);

protected:
	static int _diffU8(uint8_t a, uint8_t b);
	int _handleReceivedParam(CanasMessage * pframe, int subId, long timestamp);
	static int _handleReceivedService(CanasMessage * pmsg, int subId, long timestamp);
	virtual int _updateItem(_CanDataRegistration* curParm) = 0;

	struct _srvRequest {
		uint8_t nodeId;
		ulong 	timestamp = 0;
	};

	// structure for containing all services we can reply to
	struct _CanServiceLinks {
		int 			linkId = -1;
		service_channel_t 		canServiceCode = 0;
		ICanService* 	service = NULL;
		unsigned long 	timestamp = 0;	// last read time
		unsigned long 	maxInterval = CANAS_DEFAULT_REPEAT_TIMEOUT_MSEC;
		bool 			isAdvertised = false;
		unsigned long 	tsAdvertise = 0;	// last published time
		bool 			isPublished = false;
		bool 			isReplyOnly = false;
		bool 			isSubscribed = false;
		unsigned long 	maxIntervalAdvertiseMs = CANAS_DEFAULT_SERVICE_ADVERTISE_TIMEOUT_MSEC;
		_srvRequest*	requests = NULL;
	};

	static QList<_CanServiceLinks*> _serviceRefs;
	static int _findServiceInList(service_channel_t toFind);

	void(*_callBackData)(CAN_FRAME*);
	void(*_callBackService)(CAN_FRAME*);

	// standard service class instances

	ICanService_beacon*			_beaconService;
	ICanService_getNodeId*		_nodeIdService;
	ICanService_acceptdata*		_acceptDataService;
	ICanService_requestdata*	_requestDataService;
};
#endif
