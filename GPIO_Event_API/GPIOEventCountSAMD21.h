#ifndef EVENT_COUNTER_H
#define EVENT_COUNTER_H

#include "sam.h"
#include <stdint.h>
#include <Arduino.h>   // For Serial debugging


// -------------------- Debug Levels --------------------
#define DEBUG_NONE    0
#define DEBUG_ERROR   1
#define DEBUG_WARN    2
#define DEBUG_INFO    3
#define DEBUG_DEBUG   4
#define DEBUG_VERBOSE 5

// Select compile-time debug level
#define DEBUG_LEVEL   DEBUG_INFO

// Debug macros
#define DEBUG_PRINT(level, msg)        \
    do {                               \
        if (level <= DEBUG_LEVEL) {    \
            Serial.print(msg);         \
        }                              \
    } while (0)

#define DEBUG_PRINTLN(level, msg)      \
    do {                               \
        if (level <= DEBUG_LEVEL) {    \
            Serial.println(msg);       \
        }                              \
    } while (0)



//  User-selectable sense mode
/*  Different name is used to avoid conflict with Arduino keywords
    To use the same name- 
    // Prevent Arduino HIGH/LOW macro collisions
    #ifdef HIGH
    #undef HIGH
    #endif
    #ifdef LOW
    #undef LOW
    #endif
*/
enum class EIC_SenseMode : uint8_t {
    NOSIGNAL   = 0x0,
    EDGERISE   = 0x1,
    EDGEFALL   = 0x2,
    BOTHEDGE   = 0x3,
    LEVELHIGH  = 0x4,
    LEVELLOW   = 0x5
};

class EventCounter {
public:
    EventCounter(uint8_t port, uint8_t pin, uint8_t tcNumber, EIC_SenseMode senseMode = EIC_SenseMode::LEVELHIGH);
    void begin();
    uint32_t read();
    void reset();

private:
    uint8_t _port;
    uint8_t _pin;
    uint8_t _tcNumber;
    uint8_t _eicChannel;
    uint8_t _evsysChannel;
    uint8_t _pmuxFunc;
    EIC_SenseMode _senseMode;

    bool validateInputs();          // Validate the inputs limited to PA/PB, 0-31, TC3/4/5 while creating the instance
    bool mapEICChannel();           // Calculate EIC channels (SENSEx) for different GPIOs
    void configurePM();             // COnfigure and activate the bus/register access clocks for the peripherals
    void configureGCLK();           // Configure and activate the functional clocks for the peripherals
    void configureGPIO();           // Configure GPIO pins to use other peripheral than digital I/O
    void configureEIC();            // 
    void configureEVSYS();
    void configureTC();
};

#endif
