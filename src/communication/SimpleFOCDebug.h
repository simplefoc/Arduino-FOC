
#ifndef __SIMPLEFOCDEBUG_H__
#define __SIMPLEFOCDEBUG_H__

#include "Arduino.h"


/**
 * SimpleFOCDebug class
 * 
 * This class is used to print debug messages to a chosen output.
 * Currently, Print instances are supported as targets, e.g. serial port.
 * 
 * Activate debug output globally by calling enable(), optionally passing
 * in a Print instance. If none is provided "Serial" is used by default.
 * 
 * To produce debug output, use the macro SIMPLEFOC_DEBUG:
 *   SIMPLEFOC_DEBUG("Debug message!");
 *   SIMPLEFOC_DEBUG("a float value:", 123.456f);
 *   SIMPLEFOC_DEBUG("an integer value: ", 123);
 * 
 * Keep debugging output short and simple. Some of our MCUs have limited
 * RAM and limited serial output capabilities.
 * 
 * By default, the SIMPLEFOC_DEBUG macro uses the flash string helper to
 * help preserve memory on Arduino boards.
 * 
 * You can also disable debug output completely. In this case all debug output 
 * and the SimpleFOCDebug class is removed from the compiled code.
 * Add -DSIMPLEFOC_DISABLE_DEBUG to your compiler flags to disable debug in
 * this way.
 * 
 **/


#ifndef SIMPLEFOC_DISABLE_DEBUG

class SimpleFOCDebug {
public:
    static void enable(Print* debugPrint = &Serial);

    static void println(const __FlashStringHelper* msg);
    static void println(const char* msg);
    static void println(const __FlashStringHelper* msg, float val);
    static void println(const char* msg, float val);
    static void println(const __FlashStringHelper* msg, int val);
    static void println(const char* msg, int val);
    static void println();
    static void println(int val);
    static void println(float val);

    static void print(const char* msg);
    static void print(const __FlashStringHelper* msg);
    static void print(int val);
    static void print(float val);

protected:
    static Print* _debugPrint;
};


#define SIMPLEFOC_DEBUG(msg, ...) \
    SimpleFOCDebug::println(F(msg), ##__VA_ARGS__)





#else //ifndef SIMPLEFOC_DISABLE_DEBUG



#define SIMPLEFOC_DEBUG(msg, ...)



#endif //ifndef SIMPLEFOC_DISABLE_DEBUG
#endif

