// ----------------------------------------------------------------------------
// Constants.h
//
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <Arduino.h>
#include <MPR121.h>


namespace constants
{
extern const long baud;

extern const MPR121::DeviceAddress device_address;
extern const bool fast_mode;

extern const size_t loop_delay;

extern const uint8_t channel_count;
extern const uint8_t channel;

extern const uint8_t touch_threshold;
extern const uint8_t release_threshold;
}
#endif
