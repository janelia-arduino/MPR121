#include <Arduino.h>
#include <MPR121.h>
#include <Streaming.h>

#include "Constants.h"


MPR121 mpr121;

void setup()
{
  Serial.begin(constants::baud);

  mpr121.setupSingleDevice(Wire,
    constants::device_address,
    constants::fast_mode);
  mpr121.startChannels(constants::physical_channel_count,
    constants::proximity_mode);
  mpr121.setAllChannelsThresholds(constants::touch_threshold,
    constants::release_threshold);
}

void loop()
{
  uint8_t channel_count = mpr121.getChannelCount();
  Serial << "channel_count: " << channel_count << "\n";
  uint8_t running_channel_count = mpr121.getRunningChannelCount();
  Serial << "running_channel_count: " << running_channel_count << "\n";

  uint16_t touch_status = mpr121.getTouchStatus(constants::device_address);
  Serial << "touch_status: " << _BIN(touch_status) << "\n";
  if (mpr121.overCurrentDetected(touch_status))
  {
    Serial << "Over current detected!\n";
    mpr121.startChannels(constants::physical_channel_count,
      constants::proximity_mode);
    return;
  }
  bool any_touched = mpr121.anyTouched(touch_status);
  Serial << "any_touched: " << any_touched << "\n";
  bool device_channel_touched = mpr121.deviceChannelTouched(touch_status,constants::channel);
  Serial << "device_channel_touched: " << device_channel_touched << "\n";

  Serial << "\n";
  delay(constants::loop_delay);

  // int i;

  // if(mpr121.touchStatusChanged()) mpr121.updateTouchData();
  // mpr121.updateBaselineData();
  // mpr121.updateFilteredData();


  // Serial.print("TOUCH: ");
  // for(i=0; i<13; i++){          // 13 touch values
  //   Serial.print(mpr121.getTouchData(i), DEC);
  //   if(i<12) Serial.print(" ");
  // }
  // Serial.println();

  // Serial.print("TTHS: ");
  // for(i=0; i<13; i++){          // 13 touch thresholds
  //   Serial.print(TOUCH_THRESHOLD, DEC);
  //   if(i<12) Serial.print(" ");
  // }
  // Serial.println();

  // Serial.print("RTHS: ");
  // for(i=0; i<13; i++){          // 13 release thresholds
  //   Serial.print(RELEASE_THRESHOLD, DEC);
  //   if(i<12) Serial.print(" ");
  // }
  // Serial.println();

  // Serial.print("FDAT: ");
  // for(i=0; i<13; i++){          // 13 filtered values
  //   Serial.print(mpr121.getFilteredData(i), DEC);
  //   if(i<12) Serial.print(" ");
  // }
  // Serial.println();

  // Serial.print("BVAL: ");
  // for(i=0; i<13; i++){          // 13 baseline values
  //   Serial.print(mpr121.getBaselineData(i), DEC);
  //   if(i<12) Serial.print(" ");
  // }
  // Serial.println();

  // // the trigger and threshold values refer to the difference between
  // // the filtered data and the running baseline - see p13 of
  // // http://www.freescale.com/files/sensors/doc/data_sheet/mpr121.pdf

  // Serial.print("DIFF: ");
  // for(i=0; i<13; i++){          // 13 value pairs
  //   Serial.print(mpr121.getBaselineData(i)-mpr121.getFilteredData(i), DEC);
  //   if(i<12) Serial.print(" ");
  // }
  // Serial.println();
}
