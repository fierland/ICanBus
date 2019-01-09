//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
//
// CANdriver.h
// low level CAN driver stuff
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
#ifndef _CAN_DRIVER_H_
#define _CAN_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "XPCAN_messages.h"

#ifdef ARDUINO_ARCH_ESP32
# include <esp32_can.h>
#else
# include <can.h>
#endif

//#define XI_CANBUS_SPEED CAN_SPEED_500KBPS
#define XI_CANBUS_SPEED 500000

 /**
  * CAN ID masks
  * @{
  */
static const uint32_t CANAS_CAN_MASK_STDID = ((uint32_t)0x000007FFu);
static const uint32_t CANAS_CAN_MASK_EXTID = ((uint32_t)0x1FFFFFFFu);
/**
 * @}
 */

 /**
  * CAN flags, to be set on CAN ID
  * @{
  */
static const uint32_t CANAS_CAN_FLAG_EFF = (1 << 31);  ///< Extended frame format
static const uint32_t CANAS_CAN_FLAG_RTR = (1 << 30);  ///< Remote transmission request
/**
 * CAN frame
 */
typedef struct {
	uint8_t		data[8];
	uint32_t	id;      ///< Full ID (Standard + Extended) and flags (CANAS_CAN_FLAG_*)
	uint8_t		dlc;      ///< Data length code
} CanasCanFrame;

/**
 * Acceptance filter configuration.
 * Use flags to filter messages by type. ref CANAS_CAN_FLAG_EFF ref CANAS_CAN_FLAG_RTR.
 */
typedef struct {
	uint32_t id;
	uint32_t mask;
} CanasCanFilterConfig;

class CANdriver {
public:
	static int getDataTypeSize(uint8_t type);
	CANdriver();
	~CANdriver();

	void start(int speed = 500000, int processor = -5);
	void stop();
	void setCANPins(uint8_t pinRx, uint8_t pinTx);
	int writeMsg(CanasCanFrame* frame);

	int setFilter();
	int setFilter(int msgID, void(*callBack)(CAN_FRAME*));

	int receive(CanasCanFrame* pframe, unsigned int timeout_usec);
	int receive(CanasCanFrame* pframe);
	int receive(CanasMessage* pframe);

	static int64_t timeStamp();
	static int can2areo(CanasCanFrame* pframe, CAN_FRAME* message);
	static int areo2can(CanasCanFrame* pframe, CAN_FRAME* message);
	static int frame2msg(CanasMessage* pmsg, CanasCanFrame * pframe);
	static int msg2frame(CanasCanFrame * pframe, uint16_t msg_id, const CanasMessage* pmsg);
	static int canasHostToNetwork(uint8_t* pdata, const CanasMessageData* phost);

private:
	int _pinRx = 16;
	int _pinTx = 17;
};

#endif
