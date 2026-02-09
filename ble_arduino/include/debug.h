#ifndef DEBUG_H
#define DEBUG_H

#ifdef DEBUG_ENABLED
    #define DEBUG_SERIAL_BEGIN(x) Serial.begin(x)
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_SERIAL_BEGIN(x)
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(fmt, ...)
#endif

#endif