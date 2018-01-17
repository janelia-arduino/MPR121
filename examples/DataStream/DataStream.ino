#include <Arduino.h>
#include <Wire.h>
#include <MPR121.h>


const long BAUD = 115200;

// this is the touch threshold - setting it low makes it more like a proximity trigger
// default value is 40 for touch
const uint8_t TOUCH_THRESHOLD = 40;
// this is the release threshold - must ALWAYS be smaller than the touch threshold
// default value is 20 for touch
const uint8_t RELEASE_THRESHOLD = 20;

MPR121 mpr121;

void setup()
{
  Serial.begin(BAUD);

  if(!mpr121.begin())
  {
    Serial.println("error setting up mpr121");
    switch(mpr121.getError())
    {
      case NO_ERROR:
      {
        Serial.println("no error");
        break;
      }
      case ADDRESS_UNKNOWN:
      {
        Serial.println("incorrect address");
        break;
      }
      case READBACK_FAIL:
      {
        Serial.println("readback failure");
        break;
      }
      case OVERCURRENT_FLAG:
      {
        Serial.println("overcurrent on REXT pin");
        break;
      }
      case OUT_OF_RANGE:
      {
        Serial.println("electrode out of range");
        break;
      }
      case NOT_INITIALIZED:
      {
        Serial.println("not initialised");
        break;
      }
      default:
      {
        Serial.println("unknown error");
        break;
      }
    }
  }

  mpr121.setTouchThreshold(TOUCH_THRESHOLD);
  mpr121.setReleaseThreshold(RELEASE_THRESHOLD);
}

void loop(){
   readRawInputs();
}

void readRawInputs(){
    int i;

    if(mpr121.touchStatusChanged()) mpr121.updateTouchData();
    mpr121.updateBaselineData();
    mpr121.updateFilteredData();


    Serial.print("TOUCH: ");
    for(i=0; i<13; i++){          // 13 touch values
      Serial.print(mpr121.getTouchData(i), DEC);
      if(i<12) Serial.print(" ");
    }
    Serial.println();

    Serial.print("TTHS: ");
    for(i=0; i<13; i++){          // 13 touch thresholds
      Serial.print(TOUCH_THRESHOLD, DEC);
      if(i<12) Serial.print(" ");
    }
    Serial.println();

    Serial.print("RTHS: ");
    for(i=0; i<13; i++){          // 13 release thresholds
      Serial.print(RELEASE_THRESHOLD, DEC);
      if(i<12) Serial.print(" ");
    }
    Serial.println();

    Serial.print("FDAT: ");
    for(i=0; i<13; i++){          // 13 filtered values
      Serial.print(mpr121.getFilteredData(i), DEC);
      if(i<12) Serial.print(" ");
    }
    Serial.println();

    Serial.print("BVAL: ");
    for(i=0; i<13; i++){          // 13 baseline values
      Serial.print(mpr121.getBaselineData(i), DEC);
      if(i<12) Serial.print(" ");
    }
    Serial.println();

    // the trigger and threshold values refer to the difference between
    // the filtered data and the running baseline - see p13 of
    // http://www.freescale.com/files/sensors/doc/data_sheet/mpr121.pdf

    Serial.print("DIFF: ");
    for(i=0; i<13; i++){          // 13 value pairs
      Serial.print(mpr121.getBaselineData(i)-mpr121.getFilteredData(i), DEC);
      if(i<12) Serial.print(" ");
    }
    Serial.println();

}
