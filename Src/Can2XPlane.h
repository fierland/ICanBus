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

#ifndef _CANAS_2XPLANE_H
#define _CANAS_2XPLANE_H
#include "CanasId.h"
#include "ICanBase.h"
#include "XPCAN_messages.h"

// conversions
// fuel 6 pounds / gallon
constexpr auto XP_Conv_Fuel_poundsGallon = 6;
// 1 kilogram = 2.20462262 lbDE
constexpr auto XP_conv_kg_lb = 2.20462262;

struct CanasXplaneTrans {
	canbusId_t		canasId;
	char*			xplaneId;
	canbus_data_t	canasDataType;
};

static  CanasXplaneTrans  canasXplaneTable[] = {
		{CANAS_NOD_DEF_FUEL_TANK_1_QUANTITY,"sim/cockpit2/fuel/fuel_quantity[0]",CANAS_DATATYPE_FLOAT},  // float in Kgs
		{CANAS_NOD_DEF_FUEL_TANK_2_QUANTITY,"sim/cockpit2/fuel/fuel_quantity[1]",CANAS_DATATYPE_FLOAT},  // float in Kgs
		{CANAS_NOD_DEF_FUEL_TANK_3_QUANTITY,"sim/cockpit2/fuel/fuel_quantity[2]",CANAS_DATATYPE_FLOAT},  // float in Kgs
		{CANAS_NOD_DEF_FUEL_TANK_4_QUANTITY,"sim/cockpit2/fuel/fuel_quantity[3]",CANAS_DATATYPE_FLOAT},  // float in Kgs
		{CANAS_NOD_DEF_FUEL_TANK_5_QUANTITY,"sim/cockpit2/fuel/fuel_quantity[4]",CANAS_DATATYPE_FLOAT},  // float in Kgs
		{CANAS_NOD_DEF_FUEL_TANK_6_QUANTITY,"sim/cockpit2/fuel/fuel_quantity[5]",CANAS_DATATYPE_FLOAT},  // float in Kgs
		{CANAS_NOD_DEF_FUEL_TANK_7_QUANTITY,"sim/cockpit2/fuel/fuel_quantity[6]",CANAS_DATATYPE_FLOAT},  // float in Kgs
		{CANAS_NOD_DEF_FUEL_TANK_8_QUANTITY,"sim/cockpit2/fuel/fuel_quantity[7]",CANAS_DATATYPE_FLOAT},  // float in Kgs

		{CANAS_NOD_DEF_ENGINE_1_OIL_PRESSURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_oil_press_psi[0]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_2_OIL_PRESSURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_oil_press_psi[1]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_3_OIL_PRESSURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_oil_press_psi[2]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_4_OIL_PRESSURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_oil_press_psi[3]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_1_OIL_PRESSURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_oil_press_psi[4]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_2_OIL_PRESSURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_oil_press_psi[5]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_3_OIL_PRESSURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_oil_press_psi[6]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_4_OIL_PRESSURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_oil_press_psi[7]",CANAS_DATATYPE_FLOAT}, //psi

		{CANAS_NOD_DEF_ENGINE_1_OIL_TEMPERATURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_oil_temp_c[0]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_2_OIL_TEMPERATURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_oil_temp_c[1]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_3_OIL_TEMPERATURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_oil_temp_c[2]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_4_OIL_TEMPERATURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_oil_temp_c[3]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_1_OIL_TEMPERATURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_oil_temp_c[4]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_2_OIL_TEMPERATURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_oil_temp_c[5]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_3_OIL_TEMPERATURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_oil_temp_c[6]",CANAS_DATATYPE_FLOAT}, //psi
		{CANAS_NOD_DEF_ENGINE_4_OIL_TEMPERATURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_oil_temp_c[7]",CANAS_DATATYPE_FLOAT}, //psi

		{CANAS_NOD_DEF_FUEL_PUMP_1_FLOW_RATE,"sim/cockpit2/engine/indicators/fuel_flow_kg_sec[0]",CANAS_DATATYPE_FLOAT}, // kg/sec
		{CANAS_NOD_DEF_FUEL_PUMP_2_FLOW_RATE,"sim/cockpit2/engine/indicators/fuel_flow_kg_sec[1]",CANAS_DATATYPE_FLOAT}, // kg/sec
		{CANAS_NOD_DEF_FUEL_PUMP_3_FLOW_RATE,"sim/cockpit2/engine/indicators/fuel_flow_kg_sec[2]",CANAS_DATATYPE_FLOAT}, // kg/sec
		{CANAS_NOD_DEF_FUEL_PUMP_4_FLOW_RATE,"sim/cockpit2/engine/indicators/fuel_flow_kg_sec[3]",CANAS_DATATYPE_FLOAT}, // kg/sec
		{CANAS_NOD_DEF_FUEL_PUMP_5_FLOW_RATE,"sim/cockpit2/engine/indicators/fuel_flow_kg_sec[4]",CANAS_DATATYPE_FLOAT}, // kg/sec
		{CANAS_NOD_DEF_FUEL_PUMP_6_FLOW_RATE,"sim/cockpit2/engine/indicators/fuel_flow_kg_sec[5]",CANAS_DATATYPE_FLOAT}, // kg/sec
		{CANAS_NOD_DEF_FUEL_PUMP_7_FLOW_RATE,"sim/cockpit2/engine/indicators/fuel_flow_kg_sec[6]",CANAS_DATATYPE_FLOAT}, // kg/sec
		{CANAS_NOD_DEF_FUEL_PUMP_8_FLOW_RATE,"sim/cockpit2/engine/indicators/fuel_flow_kg_sec[7]",CANAS_DATATYPE_FLOAT}, // kg/sec

		{CANAS_NOD_DEF_ENGINE_1_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_EGT_c[0]",CANAS_DATATYPE_FLOAT}, //degrees F
		{CANAS_NOD_DEF_ENGINE_2_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_EGT_c[1]",CANAS_DATATYPE_FLOAT}, //degrees F
		{CANAS_NOD_DEF_ENGINE_3_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_EGT_c[2]",CANAS_DATATYPE_FLOAT}, //degrees F
		{CANAS_NOD_DEF_ENGINE_4_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_A,"sim/flightmodel/engine/ENGN_EGT_c[3]",CANAS_DATATYPE_FLOAT}, //degrees F
		{CANAS_NOD_DEF_ENGINE_1_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_EGT_c[4]",CANAS_DATATYPE_FLOAT}, //degrees F
		{CANAS_NOD_DEF_ENGINE_2_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_EGT_c[5]",CANAS_DATATYPE_FLOAT}, //degrees F
		{CANAS_NOD_DEF_ENGINE_3_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_EGT_c[6]",CANAS_DATATYPE_FLOAT}, //degrees F
		{CANAS_NOD_DEF_ENGINE_4_TURBINE_OUTLET_TEMPERATURE_ECS_CHANNEL_B,"sim/flightmodel/engine/ENGN_EGT_c[7]",CANAS_DATATYPE_FLOAT}, //degrees F

		{CANAS_NOD_DEF_AC_SYSTEM_1_CURRENT,"sim/cockpit2/electrical/battery_amps[0]",CANAS_DATATYPE_FLOAT}, //amps
		{CANAS_NOD_DEF_AC_SYSTEM_2_CURRENT,"sim/cockpit2/electrical/battery_amps[1]",CANAS_DATATYPE_FLOAT}, //amps
		{CANAS_NOD_DEF_AC_SYSTEM_3_CURRENT,"sim/cockpit2/electrical/battery_amps[2]",CANAS_DATATYPE_FLOAT}, //amps
		{CANAS_NOD_DEF_AC_SYSTEM_4_CURRENT,"sim/cockpit2/electrical/battery_amps[3]",CANAS_DATATYPE_FLOAT}, //amps
		{CANAS_NOD_DEF_AC_SYSTEM_5_CURRENT,"sim/cockpit2/electrical/battery_amps[4]",CANAS_DATATYPE_FLOAT}, //amps
		{CANAS_NOD_DEF_AC_SYSTEM_6_CURRENT,"sim/cockpit2/electrical/battery_amps[5]",CANAS_DATATYPE_FLOAT}, //amps
		{CANAS_NOD_DEF_AC_SYSTEM_7_CURRENT,"sim/cockpit2/electrical/battery_amps[6]",CANAS_DATATYPE_FLOAT}, //amps
		{CANAS_NOD_DEF_AC_SYSTEM_8_CURRENT,"sim/cockpit2/electrical/battery_amps[7]",CANAS_DATATYPE_FLOAT}, //amps

		{CANAS_NOD_USR_VACUUM,"sim/cockpit/misc/vacuum",CANAS_DATATYPE_FLOAT},   //psi ?
		{CANAS_NOD_USR_VACUUM2,"sim/cockpit/misc/vacuum2",CANAS_DATATYPE_FLOAT},   //psi ?

		{CANAS_NOD_USR_INSTRUMENT_LIGHT_INTENSITY,"sim/cockpit/electrical/instrument_brightness",CANAS_DATATYPE_FLOAT},
		{CANAS_NOD_USR_AVIONICS_ON,"sim/cockpit/electrical/battery_on",CANAS_DATATYPE_UCHAR},

		{0,"",0}	//zero terminator to define end of array.
};
// sim/cockpit/electrical/instrument_brightness
//==================================================================================================
//==================================================================================================
class Can2XPlane {
public:

	Can2XPlane();
	~Can2XPlane();

	static char* fromCan2Xplane(canbusId_t canID);
	static  CanasXplaneTrans* fromCan2XplaneElement(canbusId_t canID);
};

#endif
