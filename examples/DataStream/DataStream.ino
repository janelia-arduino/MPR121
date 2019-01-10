#include <Arduino.h>
#include <MPR121.h>
#include <Streaming.h>

#include "Constants.h"


MPR121 mpr121;

void setup()
{
  Serial.begin(constants::baud);

  mpr121.setupSingleDevice(Wire,constants::device_address,constants::fast_mode);
  mpr121.startChannels(constants::channel_count);

  // if(!mpr121.begin(ADDRESS))
  // {
  //   Serial.println("error setting up mpr121");
  //   switch(mpr121.getError())
  //   {
  //     case NO_ERROR:
  //     {
  //       Serial.println("no error");
  //       break;
  //     }
  //     case ADDRESS_UNKNOWN:
  //     {
  //       Serial.println("incorrect address");
  //       break;
  //     }
  //     case READBACK_FAIL:
  //     {
  //       Serial.println("readback failure");
  //       break;
  //     }
  //     case OVERCURRENT_FLAG:
  //     {
  //       Serial.println("overcurrent on REXT pin");
  //       break;
  //     }
  //     case OUT_OF_RANGE:
  //     {
  //       Serial.println("electrode out of range");
  //       break;
  //     }
  //     case NOT_INITIALIZED:
  //     {
  //       Serial.println("not initialised");
  //       break;
  //     }
  //     default:
  //     {
  //       Serial.println("unknown error");
  //       break;
  //     }
  //   }
  // }

  // mpr121.setChannelTouchThreshold(constants::channel,constants::touch_threshold);
  // mpr121.setChannelReleaseThreshold(constants::channel,constants::release_threshold);
  mpr121.setAllChannelsTouchThreshold(constants::touch_threshold);
  mpr121.setAllChannelsReleaseThreshold(constants::release_threshold);
}

void loop()
{
  uint16_t touch_status = mpr121.getTouchStatus(constants::device_address);
  Serial << "touch_status: " << _BIN(touch_status) << "\n";
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
