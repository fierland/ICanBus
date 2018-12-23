/*
 * Simple header defining macros for debug output_iterator
 */

#ifndef _ICAN_DEBUG_MACROS_H_
#define _ICAN_DEBUG_MACROS_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define NO_DEBUG 0
#define DEBUG_LEVEL 1
 //#define MACRO_DEBUG  //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.

#if DEBUG_LEVEL > 0
#define DLVARPRINT(level,txt,...) if (DEBUG_LEVEL > level) {Serial.print((char*)txt);Serial.print(__VA_ARGS__);Serial.print(":");};
#define DLVARPRINTLN(level,txt,...) if (DEBUG_LEVEL > level) {Serial.print((char*)txt);Serial.print(__VA_ARGS__);Serial.println(":");};
#define DLPRINT(level,...)  if (DEBUG_LEVEL > level) {Serial.print(__VA_ARGS__);};
#define DLPRINTLN(level,...)if (DEBUG_LEVEL > level) {Serial.println(__VA_ARGS__);};
#define DLPRINTINFO(level,...)    \
	   if (DEBUG_LEVEL > level){ \
		Serial.print(millis());     \
	   Serial.print(": ");    \
	   Serial.print(__PRETTY_FUNCTION__); \
	   Serial.print(' ');      \
	   Serial.print(__FILE__);     \
	   Serial.print(':');      \
	   Serial.print(__LINE__);     \
	   Serial.print(' ');      \
	   Serial.println(__VA_ARGS__);};
#define DLDUMP_CANFRAME(level,...)  if (DEBUG_LEVEL > level) {DumpCanFrame(__VA_ARGS__);};
#define DLDUMP_MESSAGE(level,...)  if (DEBUG_LEVEL > level) {DumpMessage(__VA_ARGS__);};
#else
#define DLVARPRINT(level,txt,...)
#define DLVARPRINTLN(level,txt,...)
#define DLPRINT(level,...)
#define DLPRINTLN(level,...)
#define DLPRINTINFO(level,...)
#define DLDUMP_CANFRAME(level,...)
#define DLDUMP_MESSAGE(level,...)
#endif

#ifdef MACRO_DEBUG    //Macros are usually in all capital letters.

#define DPRINT(...)    	Serial.print(__VA_ARGS__)    //DPRINT is a macro, debug print
#define DPRINTLN(...)  	Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#define DPRINTBUFFER(x,y) DPRINT("Buffer:");for(int i=0;i<y;i++){DPRINT(x[i]);DPRINT(":");}	DPRINTLN("END");
#define DPRINTINFO(...)    \
   Serial.print(millis());     \
   Serial.print(": ");    \
   Serial.print(__PRETTY_FUNCTION__); \
   Serial.print(' ');      \
   Serial.print(__FILE__);     \
   Serial.print(':');      \
   Serial.print(__LINE__);     \
   Serial.print(' ');      \
   Serial.println(__VA_ARGS__);
#else

#define DPRINT(...)     //now defines a blank line
#define DPRINTLN(...)   //now defines a blank line
#define DPRINTBUFFER(x,y)
#define DPRINTINFO(...)

#endif

#endif
