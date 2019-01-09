#include "Can2XPlane.h"
#include "ican_debug.h"
#include <stdlib.h>

//==================================================================================================
//==================================================================================================
Can2XPlane::Can2XPlane()
{}
//==================================================================================================
//==================================================================================================
Can2XPlane::~Can2XPlane()
{}
/*
//==================================================================================================
//==================================================================================================
uint16_t Can2XPlane::fromCan2Xplane(canbusId_t canID)
{
	DPRINTINFO("START");

	int i = 0;

	while (canasXplaneTable[i].canasId != 0 && canasXplaneTable[i].canasId != canID)i++;

	DPRINT("returning:"); DPRINTLN(i);

	DPRINTINFO("STOP");

	return canasXplaneTable[i].canasId;
}
*/
//==================================================================================================
//==================================================================================================
CanasXplaneTrans* Can2XPlane::fromCan2XplaneElement(canbusId_t canID)
{
	DPRINTINFO("START");
	CanasXplaneTrans* foundItem;

	int i = 0;

	while (canasXplaneTable[i].canasId != 0 && canasXplaneTable[i].canasId != canID)
	{
		DPRINT("CMP:"); DPRINT(canID); DPRINT("->"); DPRINTLN(canasXplaneTable[i].canasId);
		i++;
	}

	DPRINT("returning:"); DPRINTLN(i);
	foundItem = &(canasXplaneTable[i]);

	DPRINTINFO("STOP");

	return foundItem;
}
/*
CanasXplaneTrans * Can2XPlane::getName()
{
	return &getPlaneName;
}

//==================================================================================================
//==================================================================================================
int Can2XPlane::findPlane(char * planeName)
{
	int maxPlanes = sizeof(suportedPlanes) / sizeof(XplanePlanes);

	for (int i = 0; i < maxPlanes; i++)
	{
		if (strcmp(planeName, suportedPlanes[i].name))
		{
			_currentPlane = 1;
			return 0;
		}
	}
	return -1;
}
*/