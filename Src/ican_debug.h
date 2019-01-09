/*
 * Simple header defining macros for debug output_iterator
 */

#ifndef _ICAN_DEBUG_MACROS_H_
#define _ICAN_DEBUG_MACROS_H_

#include "esp_log.h"
#include "ICanBase.h"

 //#define DEBUG_TRACE

#ifdef DEBUG_TRACE
#define TRACE_START ESP_LOGV(TAG,"TRACE START: %s",__PRETTY_FUNCTION__);
#define TRACE_STOP  ESP_LOGV(TAG,"TRACE STOP: %s",__PRETTY_FUNCTION__);
#else
#define TRACE_START
#define TRACE_STOP
#endif

 //#define DUMP_CAN_FRAME(level, tag, pframe) \
 //do{\
 //ESP_LOG_LEVEL(level, tag, "CANId=%s%d %s %s DLC=%d :%2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x:",\
 //(pframe->id < 1000) ? " " : "",\
 //(pframe->id & CANAS_CAN_FLAG_EFF) ? (pframe->id & CANAS_CAN_MASK_EXTID) : (pframe->id & CANAS_CAN_MASK_STDID),\
 //(pframe->id & CANAS_CAN_FLAG_EFF) ? "EXT" : "STD",(pframe->id & CANAS_CAN_FLAG_RTR) ? "RTR" : "", pframe->dlc,\
 //pframe->data[0], pframe->data[1], pframe->data[2], pframe->data[3],pframe->data[4], pframe->data[5], pframe->data[6], pframe->data[7])\
 //} while (0);

#define DUMP_CAN_FRAME(level, tag, pframe) do { ICanBase::DumpCanFrame(pframe,level,tag); } while (0);
#define DUMP_CAN_MSG(level, tag, pmsg) do {ICanBase::DumpMessage(pmsg,level,tag);} while (0);

#define DUMP_CAN_FRAME_W(tag,pframe) DUMP_CAN_FRAME(ESP_LOG_WARN	,tag,pframe)
#define DUMP_CAN_FRAME_R(tag,pframe) DUMP_CAN_FRAME(ESP_LOG_ERROR	,tag,pframe)
#define DUMP_CAN_FRAME_I(tag,pframe) DUMP_CAN_FRAME(ESP_LOG_INFO	,tag,pframe)
#define DUMP_CAN_FRAME_D(tag,pframe) DUMP_CAN_FRAME(ESP_LOG_DEBUG	,tag,pframe)
#define DUMP_CAN_FRAME_V(tag,pframe) DUMP_CAN_FRAME(ESP_LOG_VERBOSE	,tag,pframe)

#define DUMP_CAN_MSG_W(tag,pmsg) DUMP_CAN_MSG(ESP_LOG_WARN		,tag,pmsg)
#define DUMP_CAN_MSG_E(tag,pmsg) DUMP_CAN_MSG(ESP_LOG_ERROR		,tag,pmsg)
#define DUMP_CAN_MSG_I(tag,pmsg) DUMP_CAN_MSG(ESP_LOG_INFO	,tag,pmsg)
#define DUMP_CAN_MSG_D(tag,pmsg) DUMP_CAN_MSG(ESP_LOG_DEBUG		,tag,pmsg)
#define DUMP_CAN_MSG_V(tag,pmsg) DUMP_CAN_MSG(ESP_LOG_VERBOSE	,tag,pmsg)

#define NO_DEBUG 0
#define DEBUG_LEVEL 0
 //#define MACRO_DEBUG  //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.

#ifdef NO_DEBUG_XX
typedef enum {
	LOG_NONE,       /*!< No log output */
	LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
	LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
	LOG_INFO,       /*!< Information messages which describe normal flow of events */
	LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
	LOG_VERBOSE     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
} log_level_t;

#define __SHORT_FILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

//#define DO_LOG_LEVEL(level,TAG, format, ...)  {                     \
//		char __ch ='I'; \
//        if (level==LOG_ERROR )          { __ch='E'; } \
//        else if (level==LOG_WARN )      { __ch='W'; } \
//        else if (level==LOG_DEBUG )     { __ch='D'; } \
//        else if (level==LOG_VERBOSE )   { __ch='V'; }; \
//		Serial.printf("%c (%d) %s: ",__ch,millis(),TAG); \
//		Serial.printf(format, ##__VA_ARGS__);Serial.println(""); \
//    }

//#define DO_LOG_LEVEL_LOCAL(level,TAG, format, ...) {  \
//        if ( LOG_LOCAL_LEVEL >= level ) DO_LOG_LEVEL(level, TAG, format, ##__VA_ARGS__); \
//    }

//#define DO_LOGE( format, ... ) DO_LOG_LEVEL_LOCAL(LOG_ERROR,	__SHORT_FILE__,format, ##__VA_ARGS__)
//#define DO_LOGW( format, ... ) DO_LOG_LEVEL_LOCAL(LOG_WARN,		__SHORT_FILE__,format, ##__VA_ARGS__)
//#define DO_LOGI( format, ... ) DO_LOG_LEVEL_LOCAL(LOG_INFO,		__SHORT_FILE__,format, ##__VA_ARGS__)
//#define DO_LOGD( format, ... ) DO_LOG_LEVEL_LOCAL(LOG_DEBUG,	__SHORT_FILE__,format, ##__VA_ARGS__)
//#define DO_LOGV( format, ... ) DO_LOG_LEVEL_LOCAL(LOG_VERBOSE,	__SHORT_FILE__,format, ##__VA_ARGS__)
//
//#define ESP_EARLY_LOGE(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_ERROR,TAG,format, ##__VA_ARGS__)
//#define ESP_EARLY_LOGW(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_WARN,TAG,format, ##__VA_ARGS__)
//#define ESP_EARLY_LOGD(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_DEBUG,TAG,format, ##__VA_ARGS__)
//#define ESP_EARLY_LOGV(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_VERBOSE,TAG,format, ##__VA_ARGS__)
//#define ESP_EARLY_LOGI(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_INFO,TAG,format, ##__VA_ARGS__)
//
//#define ESP_LOGE(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_ERROR,TAG,format, ##__VA_ARGS__)
//#define ESP_LOGW(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_WARN,TAG,format, ##__VA_ARGS__)
//#define ESP_LOGD(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_DEBUG,TAG,format, ##__VA_ARGS__)
//#define ESP_LOGV(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_VERBOSE,TAG,format, ##__VA_ARGS__)
//#define ESP_LOGI(TAG, format, ...) DO_LOG_LEVEL_LOCAL(LOG_INFO,TAG,format, ##__VA_ARGS__)

#define ESP_ERROR_CHECK(func) {esp_err_t err; err=func; if(err!=ESP_OK)\
	{DO_LOGE("assert failed ERR=%d in %s at line %d\n",err,__PRETTY_FUNCTION__,__LINE__);} }
#endif

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
