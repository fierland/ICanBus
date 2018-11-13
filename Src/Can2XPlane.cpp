#include "Can2XPlane.h"
#include "ican_debug.h"

Can2XPlane::Can2XPlane()
{}

Can2XPlane::~Can2XPlane()
{}

char* Can2XPlane::fromCan2Xplane(canbusId_t canID)
{
	DPRINTINFO("START");

	int i = 0;

	while (canasXplaneTable[i].canasId != 0 && canasXplaneTable[i].canasId != canID)i++;

	DPRINT("returning:"); DPRINTLN(i);

	DPRINTINFO("STOP");

	return canasXplaneTable[i].xplaneId;
}

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