//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// ICanBase.h
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
//#pragma once

#ifndef _ICANBASE_HEADER_
#define _ICANBASE_HEADER_

//#include "XPCAN_messages.h"
#include "CANdriver.h"
#include <QList.h>

// types to be used for standard iddentifiers
typedef uint16_t	canbusId_t;
typedef uint8_t		nodeId_t;
typedef uint8_t		messageId_t;
typedef uint8_t		service_channel_t;
typedef uint8_t		canbus_data_t;

// master state bit filters
constexpr uint8_t ICAN_MASTER_STATUS_INTERFACE = 1;
constexpr uint8_t ICAN_MASTER_STATUS_POWER = 2;
constexpr uint8_t ICAN_MASTER_STATUS_XPLANE_ACTIVE = 4;
/*  not used yet

constexpr uint8_t ICAN_MASTER_STATUS_ = 8;
constexpr uint8_t ICAN_MASTER_STATUS_ = 16;
constexpr uint8_t ICAN_MASTER_STATUS_ = 32;
constexpr uint8_t ICAN_MASTER_STATUS_ = 64;
constexpr uint8_t ICAN_MASTER_STATUS_ = 128;
*/

// generic callback for all subscribed parameeters to canbus
typedef int(*CanasParamCallbackFn)(float);

class ICanBase {
public:
	ICanBase();
	~ICanBase();

	virtual int stop();
	void setCANPins(uint8_t pinRx, uint8_t pinTx);

	virtual int setNodeId(nodeId_t newID);
	virtual nodeId_t getNodeId();
	//virtual int setServiceChannel(uint8_t newChannel);
	void setExternalBusState(bool isRunning);
	bool isExternalBusRunning();
	void setMasterNode(nodeId_t masterNodeId);
	uint8_t getState();
	void setState(uint8_t);

	/**
	 * Convenience functions for Node Service Protocol services.
	 * They are just wrappers over corresponding function pointers in the configuration structure.
	 * @{
	 */
	virtual uint64_t Timestamp();

	/**
 * Dump a CAN frame for humans.
 * @param [in]  pframe Pointer to frame to be dumped
 * @param [out] pbuf   Pointer to output string buffer of size @ref CANAS_DUMP_BUF_LEN
 * @return             pbuf
 */
	static void DumpCanFrame(const CanasCanFrame* pframe);

	/**
	 * Dump a CANaerospace message for humans.
	 * @param [in]  pframe Pointer to message to be dumped
	 * @param [out] pbuf   Pointer to output string buffer of size @ref CANAS_DUMP_BUF_LEN
	 * @return             pbuf
	 */
	static void DumpMessage(const CanasMessage* pmsg);
	static bool canasIsValidServiceChannel(service_channel_t service_channel);

	/**
 * Parameter publications.
 * Each parameter must be advertised before you can publish it to the bus.
 * 'interlaced' stands for traffic sharing between all available interfaces.
 * Functions of this group return @ref CanasErrorCode.
 * @{
 */
	virtual int ParamAdvertise(canbusId_t msg_id, bool addToXPlane = false, long intervalMs = 1000);
	virtual int ParamUnadvertise(canbusId_t msg_id);
	virtual int ParamPublish(canbusId_t msg_id, const CanasMessageData* pdata);
	virtual int ParamPublish(canbusId_t msg_id, float newVal);
	virtual int ParamPublish(int curRecord);
	virtual int ParamSubscribeCallback(canbusId_t msg_id, int status);

	// service requests
	virtual int ServiceSendRequest(const CanasMessage* pmsg, service_channel_t service_channel = 0);
	virtual int ServiceSendResponse(const CanasMessage* pmsg, service_channel_t service_channel = 0);

	//static void CallBack(CAN_FRAME* frame);

protected:
	uint32_t _service_request_timeout_usec = CANAS_DEFAULT_SERVICE_REQUEST_TIMEOUT_USEC; ///< Time to wait for response from remote node. Default is okay.
	uint32_t _service_poll_interval_usec = CANAS_DEFAULT_SERVICE_POLL_INTERVAL_USEC;  ///< Do not change
//	uint8_t  _service_frame_hist_len = CANAS_DEFAULT_SERVICE_HIST_LEN;  ///< Do not change
	uint32_t _repeat_timeout_usec = CANAS_DEFAULT_REPEAT_TIMEOUT_USEC;///< Largest interval of repeated messages (default should be good enough)

	static const int CANAS_DEFAULT_REPEAT_TIMEOUT_USEC = 30 * 1000 * 1000;

	static int serviceChannelToMessageID(service_channel_t service_channel, bool isrequest);

	static MessageGroup _detectMessageGroup(canbusId_t id);

	struct _CanSrvReq {
		nodeId_t node_id = 0;
		uint8_t	service_channel = 0;
		ulong	timestamp = 0;
	};

	static QList<_CanSrvReq*> _serviceReqQueue;
	//static int _findDataRegistrationInList(uint16_t toFind);

	// structure for containing all published dataelements we publish
	// TODO: Check if we need 2 structs for this
	//struct _CanPublishLinks {
	//	int linkId;
	//	canbusId_t canAreoId = 0;
	//	float	currentVal = 0;
	//	uint32_t maxIntervalMs = 100;
	//	uint8_t last_message_code = 0;
	//	ulong timestamp = 0;	// last send time
	//	CanasMessageData data;
	//	bool isXplane = false;
	//};

	typedef struct _CanDataRegistration {
		canbusId_t canAreoId = 0;
		int		replyCode = -1;
		int		linkId;
		float	lastVal;
		void*	indicator = NULL;
		ulong	timestamp = 0;  // last read time
		ulong	maxInterval = CANAS_DEFAULT_REPEAT_TIMEOUT_USEC;
		// publush info
		bool	isAdvertised = false;
		ulong	tsPublish = 0;	// last send time
		ulong	maxIntervalUs = 100;
		CanasMessageData data;
		bool	isXplane = false;
		// subsription info
		bool	isSubscribed = false;
		ulong	tsReceived = 0;
		bool	gotReply = false;
		bool	doUpdate = false;
		messageId_t last_message_code = 0;
		// advertisement info
	};

	static QList<_CanDataRegistration*> _listCanDataRegistrations;
	static int _findDataRegistrationInList(uint16_t toFind);

	int _genericSend(canbusId_t msg_id, uint8_t msggroup, const CanasMessage* pmsg);

	static int _canasDataCopy(CanasMessageData * pdataTo, const CanasMessageData * pdataFrom);

	static int _parseFrame(const CanasCanFrame* pframe, uint16_t* pmsg_id, CanasMessage* pmsg);

	// internal values
	CANdriver	_canBus;
	nodeId_t	_masterNodeId = 0;
	bool		_running = false;
	static nodeId_t	_node_id;			///< Local Node ID
	bool _externalBusIsRunning = false;
	bool _instrumentPowerIsOn = false;
	bool _masterRunning = false;
	int _speed = XI_CANBUS_SPEED;

	// callbacks for X Plane
	int(*_newXservicecall)(canbusId_t) = NULL;
	int(*_removeXservicecall)(canbusId_t) = NULL;
	int(*_changeXPvalueCall)(canbusId_t, float) = NULL;

private:
};

#endif // _ICANBASE_HEADER_
