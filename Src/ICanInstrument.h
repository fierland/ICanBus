//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// ICanInstrument.h
// Main Can protocol class specific for instruments.
//
// VERSION HISTORY:
//
//==================================================================================================
// A light implementation of the CAN Aerospace protocol to manage simulated instruments.
//	This code is in the public domain.
//
// Thanks to mjs513/CANaerospace (Pavel Kirienko, 2013 (pavel.kirienko@gmail.com))
//-------------------------------------------------------------------------------------------------------------------

#ifndef _ICAN_INSTRUMENT_H_
#define _ICAN_INSTRUMENT_H_

#ifdef ICAN_INSTRUMENT
#include "ICanBaseSrv.h"
#include "GenericIndicator.h"

class ICanInstrument : public ICanBaseSrv {
public:
	ICanInstrument(uint8_t hdwId, uint8_t swId, void(*myCallBack)(CAN_FRAME *) = NULL, void(*myCallBackService)(CAN_FRAME *) = NULL);
	~ICanInstrument();

	int start(int speed = 500000, int toCore = 0);
	int stop();
	void updateInstrument();  // main proces loop

	/// <summary>
	/// Parameter subscriptions. Each parameter must be subscribed before you can read it from the
	/// bus. Each subscription can track an arbitrary number of redundant units transmitting this
	/// message, from 1 to 255.
	/// </summary>
	/// <param name="msg_id">canas message id , see canas manual</param>
	/// <param name="instrument">pointer to instruement class to display parameter</param>
	/// <returns>CanasErrorCode</returns>
	//	int ParamRegister(GenericIndicator* instrument, bool subscribe = true);
	int ParamRegister(const uint16_t canId, const int interval, const bool send);
	//int ParamRegister(queueDataSetItem* dataItem);

protected:
	TaskHandle_t xTaskCanbus = NULL;
	bool _firstStart = true;

	//int _connect2master();
	//int _updateValue(uint16_t type, float value);
	int _updateItem(_CanDataRegistration * curParm);
	int _checkNewRegistrations(long timestamp);
	int _checkCanBus(long timestamp);
	int _checkDataRegistrations(long timestamp);
	int _checkAdvertisements(long timestamp);
	// structure for containing the subscribed data elements to capture
};
#endif
#endif