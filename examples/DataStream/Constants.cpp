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
const bool fast_mode = true;

const size_t loop_delay = 500;

const uint8_t physical_channel_count = 2;
const MPR121::ProximityMode proximity_mode = MPR121::COMBINE_CHANNELS_0_TO_1;

const uint8_t channel = 0;

// this is the touch threshold - setting it low makes it more like a proximity trigger
// default value is 40 for touch
const uint8_t touch_threshold = 40;
// this is the release threshold - must ALWAYS be smaller than the touch threshold
// default value is 20 for touch
const uint8_t release_threshold = 20;
}
