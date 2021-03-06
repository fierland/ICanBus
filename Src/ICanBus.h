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

#ifndef _ICAN_BUS_H_
#define _ICAN_BUS_H_

#include <ICanBaseSrv.h>

#ifdef ICAN_MASTER

class ICanBus : public ICanBaseSrv {
public:
	/// <summary>
	/// Initialize class instance.
	/// </summary>
	/// <param name="nodeId">        nodeId to use</param>
	/// <param name="serviceChannel"> service channel to use</param>
	/// <param name="hdwId">         Hardware revision</param>
	/// <param name="swId">          software revision</param>

	ICanBus(uint8_t nodeId = 255, uint8_t hdwId = 0, uint8_t swId = 0, void(*myCallBack)(CAN_FRAME*) = NULL, void(*myCallBackService)(CAN_FRAME*) = NULL);
	~ICanBus();
	int start(int speed = 500000, int core = 1);
	int stop();

	/**
	 * Update instance state.
	 * Must be called for every new incoming frame or by timeout.
	 * @note It is recommended to call this function at least every 10 ms
	 * @return            @ref CanasErrorCode
	 */
	int UpdateMaster();

	/// <summary>
	/// Parameter subscriptions. Each parameter must be subscribed before you can read it from the
	/// bus. Each subscription can track an arbitrary number of redundant units transmitting this
	/// message, from 1 to 255.
	/// </summary>
	/// <param name="msg_id">canas message id , see canas manual</param>
	/// <param name="instrument">pointer to instruement class to display parameter</param>
	/// <returns>CanasErrorCode</returns>
	int ParamRegister(canbusId_t msg_id, bool subscribe = false);
	int ParamUpdateValue(canbusId_t msg_id, CanasMessageData data);

	int checkNewDataFromXP();

protected:

	int checkAdvertisements(long timestamp);
	int checkDataRefs(long timestamp);

	// structure for containing the subscribed data elements to capture
	int _updateItem(_CanDataRegistration * curParm);

	TaskHandle_t xTaskCanbus = NULL;
	bool _firstStart = true;

private:
};

#endif
#endif