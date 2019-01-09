//==================================================================================================
//  Franks Flightsim Intruments project
//  by Frank van Ierland
//
// This code is in the public domain.
//
//==================================================================================================
// canas2Xplane.h
/// <summary>
/// Tranlation object from Canas codes to Xplane strings
/// </summary>
//==================================================================================================

#ifndef __CANAS_2XPLANE_H
#define __CANAS_2XPLANE_H
#include <stdlib.h>
#include "CanasId.h"
#include "XPCAN_messages.h"

// conversions
// fuel 6 pounds / gallon
constexpr auto XP_Conv_Fuel_poundsGallon = 6;
// 1 kilogram = 2.20462262 lbDE
constexpr auto XP_conv_kg_lb = 2.20462262;

struct CanasXplaneTrans {
	canbusId_t		canasId;
	canbus_data_t	canasDataType;
	int				canIntervalMs;
};

static CanasXplaneTrans canasXplaneTable[MAX_XP_CAN_ITEMS]{
	{CANAS_NOD_DEF_172_FUEL_L,CANAS_DATATYPE_FLOAT,10000},
	{CANAS_NOD_DEF_172_FUEL_R,CANAS_DATATYPE_FLOAT,10000},
	{CANAS_NOD_DEF_172_EGT,CANAS_DATATYPE_FLOAT,10000},
	{CANAS_NOD_DEF_172_FUEL_FLOW,CANAS_DATATYPE_FLOAT,10000},
	{CANAS_NOD_DEF_172_BAT_AMPS,CANAS_DATATYPE_FLOAT,10000},
	{CANAS_NOD_DEF_172_SUCTION,CANAS_DATATYPE_FLOAT,10000},
	{CANAS_NOD_DEF_172_OIL_TMP,CANAS_DATATYPE_FLOAT,10000},
	{CANAS_NOD_DEF_172_OIL_PRES,CANAS_DATATYPE_FLOAT,10000},
	{CANAS_NOD_USR_INSTRUMENT_LIGHT_INTENSITY,CANAS_DATATYPE_FLOAT,10000},
	{CANAS_NOD_USR_AVIONICS_ON,CANAS_DATATYPE_UCHAR,20000},
	{0,0,0}
};

//==================================================================================================
//==================================================================================================
class Can2XPlane {
public:

	Can2XPlane();
	~Can2XPlane();

	//static char* fromCan2Xplane(canbusId_t canID);
	static  CanasXplaneTrans* fromCan2XplaneElement(canbusId_t canID);
	//static  CanasXplaneTrans* getName();
	//static int findPlane(char* planeName);
private:
	//static int _currentPlane;
};

#endif
