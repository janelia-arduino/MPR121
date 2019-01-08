// ----------------------------------------------------------------------------
// Constants.cpp
//
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#include "Constants.h"


namespace constants
{
const long baud = 115200;
const MPR121::DeviceAddress device_address = MPR121::ADDRESS_5A;

const size_t loop_delay = 2000;

const uint8_t channel_count = 1;

// this is the touch threshold - setting it low makes it more like a proximity trigger
// default value is 40 for touch
const uint8_t touch_threshold = 40;
// this is the release threshold - must ALWAYS be smaller than the touch threshold
// default value is 20 for touch
const uint8_t release_threshold = 20;
}
