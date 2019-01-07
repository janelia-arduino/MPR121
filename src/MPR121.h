// ----------------------------------------------------------------------------
// MPR121.h
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// Jim Lindblom
// Stefan Dzisiewski-Smith
// Peter Krige
// Adafruit <info@adafruit.com>
// ----------------------------------------------------------------------------
#ifndef MPR121_H
#define MPR121_H
#include <Arduino.h>
#include <Wire.h>


class MPR121
{
public:
  MPR121();

  enum DeviceAddress
    {
     ADDRESS_5A = 0x5A,
     ADDRESS_5B = 0x5B,
     ADDRESS_5C = 0x5C,
     ADDRESS_5D = 0x5D,
    };

  // Convenience method when using a single device
  bool setupSingleDevice(TwoWire & wire=Wire,
    DeviceAddress device_address=ADDRESS_5A,
    bool fast_mode=false);

  // Methods for using multiple devices
  // Take care when using fast_mode_plus with non-PCA9685 devices
  void setWire(TwoWire & wire=Wire,
    bool fast_mode=false);

  void addDevice(DeviceAddress device_address);
  bool resetAllDevices();

  void enableDeviceChannels(DeviceAddress device_address,
    uint8_t number_of_channels_enabled);
  void stop(DeviceAddress device_address);
  bool isRunning(DeviceAddress device_address);

  // // getError() returns an Error indicating the current
  // // error on the MPR121 - clearError() clears this
  // // error codes
  // enum Error
  //   {
  //    NO_ERROR, // no error
  //    RETURN_TO_SENDER, // not implemented
  //    ADDRESS_UNKNOWN, // no MPR121 found at specified I2C address
  //    READBACK_FAIL, // readback from MPR121 was not as expected
  //    OVERCURRENT_FLAG, // overcurrent on REXT pin
  //    OUT_OF_RANGE, // autoconfiguration fail, often a result of shorted pins
  //    NOT_INITIALIZED, // device has not been initialised
  //   };
  // Error getError();
  // void clearError();

  // // updates the data from the MPR121 into our internal buffer
  // // updateTouchData() does this only for touch on / off status
  // // updateBaseLineData() does this for background baseline
  // // updateFilteredData() does this for continuous proximity data
  // // updateAll() does all three

  // // the appropriate function from these must be called before data
  // // from touched(), getFilteredData() etc. can be considered
  // // valid
  // void updateTouchData();
  // bool updateBaselineData();
  // bool updateFilteredData();
  // void updateAll();

  // bool anyTouched();

  // // returns a boolean indicating the touch status of a given electrode
  // bool touched(const uint8_t electrode);

  // // returns the number of touches currently detected
  // uint8_t getTouchCount();

  // // returns continous proximity or baseline data for a given electrode
  // int getBaselineData(const uint8_t electrode);
  // int getFilteredData(const uint8_t electrode);

  // // returns boolean indicating whether a new touch or release has been
  // // detected since the last time updateTouchData() was called
  // bool isNewTouch(const uint8_t electrode);
  // bool isNewRelease(const uint8_t electrode);

  // // sets touch and release thresholds either for all electrodes, or
  // // for a specfic electrode - higher values = less sensitive and
  // // release threshold must ALWAYS be lower than touch threshold
  // void setTouchThreshold(const uint8_t threshold);
  // void setTouchThreshold(const uint8_t electrode, const uint8_t threshold);
  // void setReleaseThreshold(const uint8_t threshold);
  // void setReleaseThreshold(const uint8_t electrode, const uint8_t threshold);

  // // returns the current touch or release threshold for a specified electrode
  // uint8_t getTouchThreshold(const uint8_t electrode);
  // uint8_t getReleaseThreshold(const uint8_t electrode);

  // // ------------------ ADVANCED FUNCTIONS ------------------

  // // set number of electrodes to use to generate virtual "13th"
  // // proximity electrode
  // // see http://cache.freescale.com/files/sensors/doc/app_note/AN3893.pdf
  // //
  // // N.B. - this is not related to general proximity detection or
  // // reading back continuous proximity data
  // // "13th electrode" proximity modes
  // // N.B. this does not relate to normal proximity detection
  // // see http://cache.freescale.com/files/sensors/doc/app_note/AN3893.pdf
  // enum ProximityMode
  //   {
  //    DISABLED, // proximity mode disabled
  //    PROX0_1, // proximity mode for ELE0..ELE1
  //    PROX0_3, // proximity mode for ELE0..ELE3
  //    PROX0_11, // proximity mode for ELE0..ELE11
  //   };
  // void setProximityMode(const ProximityMode mode);

  // // Enables GPIO mode for up to 8 of the MPR121 electrodes
  // // starts with electrode 11 - i.e. setNumDigPins(1) sets just
  // // electrode 11 as GPIO, setNumDigPins(2) sets electrodes 11
  // // & 10 as GPIO, and so on. Electrodes 0 to 3 cannot be used
  // // as GPIO
  // //
  // // N.B. electrodes are 3.3V and WILL be damaged if driven by
  // // a greater voltage
  // void setDigitalPinCount(const uint8_t pin_count);

  // // Sets pin mode for an electrode already set as GPIO by
  // // setDigitalPinCount() - see section "GPIO pin function constants"
  // // for details
  // // GPIO pin function constants
  // enum PinMode
  //   {
  //    // INPUT and OUTPUT are already defined by Arduino, use its definitions

  //    //INPUT, // digital input
  //    INPUT_PU, // digital input with pullup
  //    INPUT_PD, // digital input with pulldown
  //    //OUTPUT, // digital output (push-pull)
  //    OUTPUT_HS, // digital output, open collector (high side)
  //    OUTPUT_LS, // digital output, open collector (low side)
  //   };
  // void pinMode(const uint8_t electrode, const PinMode mode);
  // void pinMode(const uint8_t electrode, int mode);

  // // Similar to digitalWrite in Arduino for GPIO electrode
  // void digitalWrite(uint8_t electrode, uint8_t val);

  // // Toggles electrode set as GPIO output
  // void digitalToggle(uint8_t electrode);

  // // Reads electrode set as GPIO input
  // bool digitalRead(uint8_t electrode);

  // // Writes PWM value to electrode set as GPIO output - very limited
  // // (4 bit, although input to function is 0..255 to match Arduino,
  // // internally reduced to 4 bit) and broken on ELE9 and ELE10
  // // see https://community.freescale.com/thread/305474
  // void analogWrite(uint8_t electrode, uint8_t val);

  // // Sets the sample period of the MPR121 - the time between capacitive
  // // readings. Higher values consume less power, but are less responsive.
  // // sample intervals
  // enum SamplePeriod
  //   {
  //    SAMPLE_PERIOD_1MS = 0x00,
  //    SAMPLE_PERIOD_2MS = 0x01,
  //    SAMPLE_PERIOD_4MS = 0x02,
  //    SAMPLE_PERIOD_8MS = 0x03,
  //    SAMPLE_PERIOD_16MS = 0x04,
  //    SAMPLE_PERIOD_32MS = 0x05,
  //    SAMPLE_PERIOD_64MS = 0x06,
  //    SAMPLE_PERIOD_128MS = 0x07
  //   };
  // void setSamplePeriod(SamplePeriod period);

private:
  enum {DEVICE_COUNT_MAX=4};
  uint8_t device_count_;
  DeviceAddress device_addresses_[DEVICE_COUNT_MAX];

  TwoWire * wire_ptr_;
  const static long FAST_MODE_CLOCK_FREQUENCY = 400000;

  const static int DEVICE_INDEX_NONE = -1;
  int deviceAddressToDeviceIndex(DeviceAddress device_address);

  const static bool SUCCESS = true;

  bool reset(DeviceAddress device_address);

  template<typename T>
  void write(DeviceAddress device_address,
    uint8_t register_address,
    T data);
  template<typename T>
  void read(DeviceAddress device_address,
    uint8_t register_address,
    T & data);

  const static uint8_t BITS_PER_BYTE = 8;
  const static uint8_t BITS_PER_TWO_BYTES = 16;
  const static uint8_t BYTE_MAX = 0xFF;
  const static uint16_t TWO_BYTE_MAX = 0xFFFF;

  // const static uint8_t NOT_INITIALIZED_BIT = 0;
  // const static uint8_t ADDRESS_UNKNOWN_BIT = 1;
  // const static uint8_t READBACK_FAIL_BIT = 2;
  // const static uint8_t OVERCURRENT_FLAG_BIT = 3;
  // const static uint8_t OUT_OF_RANGE_BIT = 4;

  // const static uint8_t ELECTRODE_COUNT = 13;

  // const static uint8_t DIGITAL_PIN_COUNT_MAX = 8;

  // // registers
  // // touch and OOR statuses
  // const static uint8_t TSL_REGISTER_ADDRESS = 0x00;
  // const static uint8_t TSH_REGISTER_ADDRESS = 0x01;
  // const static uint8_t OORSL_REGISTER_ADDRESS = 0x02;
  // const static uint8_t OORSH_REGISTER_ADDRESS = 0x03;

  // // filtered data
  // const static uint8_t E0FDL_REGISTER_ADDRESS = 0x04;
  // const static uint8_t E0FDH_REGISTER_ADDRESS = 0x05;
  // const static uint8_t E1FDL_REGISTER_ADDRESS = 0x06;
  // const static uint8_t E1FDH_REGISTER_ADDRESS = 0x07;
  // const static uint8_t E2FDL_REGISTER_ADDRESS = 0x08;
  // const static uint8_t E2FDH_REGISTER_ADDRESS = 0x09;
  // const static uint8_t E3FDL_REGISTER_ADDRESS = 0x0A;
  // const static uint8_t E3FDH_REGISTER_ADDRESS = 0x0B;
  // const static uint8_t E4FDL_REGISTER_ADDRESS = 0x0C;
  // const static uint8_t E4FDH_REGISTER_ADDRESS = 0x0D;
  // const static uint8_t E5FDL_REGISTER_ADDRESS = 0x0E;
  // const static uint8_t E5FDH_REGISTER_ADDRESS = 0x0F;
  // const static uint8_t E6FDL_REGISTER_ADDRESS = 0x10;
  // const static uint8_t E6FDH_REGISTER_ADDRESS = 0x11;
  // const static uint8_t E7FDL_REGISTER_ADDRESS = 0x12;
  // const static uint8_t E7FDH_REGISTER_ADDRESS = 0x13;
  // const static uint8_t E8FDL_REGISTER_ADDRESS = 0x14;
  // const static uint8_t E8FDH_REGISTER_ADDRESS = 0x15;
  // const static uint8_t E9FDL_REGISTER_ADDRESS = 0x16;
  // const static uint8_t E9FDH_REGISTER_ADDRESS = 0x17;
  // const static uint8_t E10FDL_REGISTER_ADDRESS = 0x18;
  // const static uint8_t E10FDH_REGISTER_ADDRESS = 0x19;
  // const static uint8_t E11FDL_REGISTER_ADDRESS = 0x1A;
  // const static uint8_t E11FDH_REGISTER_ADDRESS = 0x1B;
  // const static uint8_t E12FDL_REGISTER_ADDRESS = 0x1C;
  // const static uint8_t E12FDH_REGISTER_ADDRESS = 0x1D;

  // // baseline values
  // const static uint8_t E0BV_REGISTER_ADDRESS = 0x1E;
  // const static uint8_t E1BV_REGISTER_ADDRESS = 0x1F;
  // const static uint8_t E2BV_REGISTER_ADDRESS = 0x20;
  // const static uint8_t E3BV_REGISTER_ADDRESS = 0x21;
  // const static uint8_t E4BV_REGISTER_ADDRESS = 0x22;
  // const static uint8_t E5BV_REGISTER_ADDRESS = 0x23;
  // const static uint8_t E6BV_REGISTER_ADDRESS = 0x24;
  // const static uint8_t E7BV_REGISTER_ADDRESS = 0x25;
  // const static uint8_t E8BV_REGISTER_ADDRESS = 0x26;
  // const static uint8_t E9BV_REGISTER_ADDRESS = 0x27;
  // const static uint8_t E10BV_REGISTER_ADDRESS = 0x28;
  // const static uint8_t E11BV_REGISTER_ADDRESS = 0x29;
  // const static uint8_t E12BV_REGISTER_ADDRESS = 0x2A;

  // // general electrode touch sense baseline filters
  // // rising filter
  // const static uint8_t MHDR_REGISTER_ADDRESS = 0x2B;
  // const static uint8_t NHDR_REGISTER_ADDRESS = 0x2C;
  // const static uint8_t NCLR_REGISTER_ADDRESS = 0x2D;
  // const static uint8_t FDLR_REGISTER_ADDRESS = 0x2E;

  // // falling filter
  // const static uint8_t MHDF_REGISTER_ADDRESS = 0x2F;
  // const static uint8_t NHDF_REGISTER_ADDRESS = 0x30;
  // const static uint8_t NCLF_REGISTER_ADDRESS = 0x31;
  // const static uint8_t FDLF_REGISTER_ADDRESS = 0x32;

  // // touched filter
  // const static uint8_t NHDT_REGISTER_ADDRESS = 0x33;
  // const static uint8_t NCLT_REGISTER_ADDRESS = 0x34;
  // const static uint8_t FDLT_REGISTER_ADDRESS = 0x35;

  // // proximity electrode touch sense baseline filters
  // // rising filter
  // const static uint8_t MHDPROXR_REGISTER_ADDRESS = 0x36;
  // const static uint8_t NHDPROXR_REGISTER_ADDRESS = 0x37;
  // const static uint8_t NCLPROXR_REGISTER_ADDRESS = 0x38;
  // const static uint8_t FDLPROXR_REGISTER_ADDRESS = 0x39;

  // // falling filter
  // const static uint8_t MHDPROXF_REGISTER_ADDRESS = 0x3A;
  // const static uint8_t NHDPROXF_REGISTER_ADDRESS = 0x3B;
  // const static uint8_t NCLPROXF_REGISTER_ADDRESS = 0x3C;
  // const static uint8_t FDLPROXF_REGISTER_ADDRESS = 0x3D;

  // // touched filter
  // const static uint8_t NHDPROXT_REGISTER_ADDRESS = 0x3E;
  // const static uint8_t NCLPROXT_REGISTER_ADDRESS = 0x3F;
  // const static uint8_t FDLPROXT_REGISTER_ADDRESS = 0x40;

  // // electrode touch and release thresholds
  // const static uint8_t E0TTH_REGISTER_ADDRESS = 0x41;
  // const static uint8_t E0RTH_REGISTER_ADDRESS = 0x42;
  // const static uint8_t E1TTH_REGISTER_ADDRESS = 0x43;
  // const static uint8_t E1RTH_REGISTER_ADDRESS = 0x44;
  // const static uint8_t E2TTH_REGISTER_ADDRESS = 0x45;
  // const static uint8_t E2RTH_REGISTER_ADDRESS = 0x46;
  // const static uint8_t E3TTH_REGISTER_ADDRESS = 0x47;
  // const static uint8_t E3RTH_REGISTER_ADDRESS = 0x48;
  // const static uint8_t E4TTH_REGISTER_ADDRESS = 0x49;
  // const static uint8_t E4RTH_REGISTER_ADDRESS = 0x4A;
  // const static uint8_t E5TTH_REGISTER_ADDRESS = 0x4B;
  // const static uint8_t E5RTH_REGISTER_ADDRESS = 0x4C;
  // const static uint8_t E6TTH_REGISTER_ADDRESS = 0x4D;
  // const static uint8_t E6RTH_REGISTER_ADDRESS = 0x4E;
  // const static uint8_t E7TTH_REGISTER_ADDRESS = 0x4F;
  // const static uint8_t E7RTH_REGISTER_ADDRESS = 0x50;
  // const static uint8_t E8TTH_REGISTER_ADDRESS = 0x51;
  // const static uint8_t E8RTH_REGISTER_ADDRESS = 0x52;
  // const static uint8_t E9TTH_REGISTER_ADDRESS = 0x53;
  // const static uint8_t E9RTH_REGISTER_ADDRESS = 0x54;
  // const static uint8_t E10TTH_REGISTER_ADDRESS = 0x55;
  // const static uint8_t E10RTH_REGISTER_ADDRESS = 0x56;
  // const static uint8_t E11TTH_REGISTER_ADDRESS = 0x57;
  // const static uint8_t E11RTH_REGISTER_ADDRESS = 0x58;
  // const static uint8_t E12TTH_REGISTER_ADDRESS = 0x59;
  // const static uint8_t E12RTH_REGISTER_ADDRESS = 0x5A;

  // // debounce settings
  // const static uint8_t DTR_REGISTER_ADDRESS = 0x5B;

  // // configuration registers
  const static uint8_t CDC_REGISTER_ADDRESS = 0x5C;
  const static uint8_t CDC_REGISTER_DEFAULT = 0x10;
  const static uint8_t CDT_REGISTER_ADDRESS = 0x5D;
  const static uint8_t CDT_REGISTER_DEFAULT = 0x24;
  const static uint8_t ECR_REGISTER_ADDRESS = 0x5E;
  struct ElectrodeConfiguration
  {
    uint8_t electrode_enable : 4;
    uint8_t proximity_enable : 2;
    uint8_t calibration_lock : 2;
  };

  // // electrode currents
  // const static uint8_t CDC0_REGISTER_ADDRESS = 0x5F;
  // const static uint8_t CDC1_REGISTER_ADDRESS = 0x60;
  // const static uint8_t CDC2_REGISTER_ADDRESS = 0x61;
  // const static uint8_t CDC3_REGISTER_ADDRESS = 0x62;
  // const static uint8_t CDC4_REGISTER_ADDRESS = 0x63;
  // const static uint8_t CDC5_REGISTER_ADDRESS = 0x64;
  // const static uint8_t CDC6_REGISTER_ADDRESS = 0x65;
  // const static uint8_t CDC7_REGISTER_ADDRESS = 0x66;
  // const static uint8_t CDC8_REGISTER_ADDRESS = 0x67;
  // const static uint8_t CDC9_REGISTER_ADDRESS = 0x68;
  // const static uint8_t CDC10_REGISTER_ADDRESS = 0x69;
  // const static uint8_t CDC11_REGISTER_ADDRESS = 0x6A;
  // const static uint8_t CDCPROX_REGISTER_ADDRESS = 0x6B;

  // // electrode charge times
  // const static uint8_t CDT01_REGISTER_ADDRESS = 0x6C;
  // const static uint8_t CDT23_REGISTER_ADDRESS = 0x6D;
  // const static uint8_t CDT45_REGISTER_ADDRESS = 0x6E;
  // const static uint8_t CDT67_REGISTER_ADDRESS = 0x6F;
  // const static uint8_t CDT89_REGISTER_ADDRESS = 0x70;
  // const static uint8_t CDT1011_REGISTER_ADDRESS = 0x71;
  // const static uint8_t CDTPROX_REGISTER_ADDRESS = 0x72;

  // // GPIO
  // const static uint8_t CTL0_REGISTER_ADDRESS = 0x73;
  // const static uint8_t CTL1_REGISTER_ADDRESS = 0x74;
  // const static uint8_t DAT_REGISTER_ADDRESS = 0x75;
  // const static uint8_t DIR_REGISTER_ADDRESS = 0x76;
  // const static uint8_t EN_REGISTER_ADDRESS = 0x77;
  // const static uint8_t SET_REGISTER_ADDRESS = 0x78;
  // const static uint8_t CLR_REGISTER_ADDRESS = 0x79;
  // const static uint8_t TOG_REGISTER_ADDRESS = 0x7A;

  // // auto-config
  // const static uint8_t ACCR0_REGISTER_ADDRESS = 0x7B;
  // const static uint8_t ACCR1_REGISTER_ADDRESS = 0x7C;
  // const static uint8_t USL_REGISTER_ADDRESS = 0x7D;
  // const static uint8_t LSL_REGISTER_ADDRESS = 0x7E;
  // const static uint8_t TL_REGISTER_ADDRESS = 0x7F;

  // soft reset
  const static uint8_t SRST_REGISTER_ADDRESS = 0x80;
  const static uint8_t SRST_REGISTER_VALUE = 0x63;

  // // PWM
  // const static uint8_t PWM0 = 0x81;
  // const static uint8_t PWM1 = 0x82;
  // const static uint8_t PWM2 = 0x83;
  // const static uint8_t PWM3 = 0x84;

  // struct Settings
  // {
  //   // touch and release thresholds
  //   uint8_t TTHRESH;
  //   uint8_t RTHRESH;

  //   // general electrode touch sense baseline filters
  //   // rising filter
  //   uint8_t MHDR;
  //   uint8_t NHDR;
  //   uint8_t NCLR;
  //   uint8_t FDLR;

  //   // falling filter
  //   uint8_t MHDF;
  //   uint8_t NHDF;
  //   uint8_t NCLF;
  //   uint8_t FDLF;

  //   // touched filter
  //   uint8_t NHDT;
  //   uint8_t NCLT;
  //   uint8_t FDLT;

  //   // proximity electrode touch sense baseline filters
  //   // rising filter
  //   uint8_t MHDPROXR;
  //   uint8_t NHDPROXR;
  //   uint8_t NCLPROXR;
  //   uint8_t FDLPROXR;

  //   // falling filter
  //   uint8_t MHDPROXF;
  //   uint8_t NHDPROXF;
  //   uint8_t NCLPROXF;
  //   uint8_t FDLPROXF;

  //   // touched filter
  //   uint8_t NHDPROXT;
  //   uint8_t NCLPROXT;
  //   uint8_t FDLPROXT;

  //   // debounce settings
  //   uint8_t DTR;

  //   // configuration registers
  //   uint8_t AFE1;
  //   uint8_t AFE2;
  //   uint8_t ECR;

  //   // auto-configuration registers
  //   uint8_t ACCR0;
  //   uint8_t ACCR1;
  //   uint8_t USL;
  //   uint8_t LSL;
  //   uint8_t TL;

  //   // default values in initialisation list
  //   Settings():
  //     TTHRESH(40),
  //     RTHRESH(20),
  //     MHDR(0x01),
  //     NHDR(0x01),
  //     NCLR(0x10),
  //     FDLR(0x20),
  //     MHDF(0x01),
  //     NHDF(0x01),
  //     NCLF(0x10),
  //     FDLF(0x20),
  //     NHDT(0x01),
  //     NCLT(0x10),
  //     FDLT(0xFF),
  //     MHDPROXR(0x0F),
  //     NHDPROXR(0x0F),
  //     NCLPROXR(0x00),
  //     FDLPROXR(0x00),
  //     MHDPROXF(0x01),
  //     NHDPROXF(0x01),
  //     NCLPROXF(0xFF),
  //     FDLPROXF(0xFF),
  //     NHDPROXT(0x00),
  //     NCLPROXT(0x00),
  //     FDLPROXT(0x00),
  //     DTR(0x11),
  //     AFE1(0xFF),
  //     AFE2(0x30),
  //     ECR(0xCC), // default to fast baseline startup and 12 electrodes enabled, no prox
  //     ACCR0(0x00),
  //     ACCR1(0x00),
  //     USL(0x00),
  //     LSL(0x00),
  //     TL(0x00),
  //   {};
  // };
  // const static Settings default_settings_;

  // TwoWire * wire_ptr_;
  // uint8_t address_;
  // uint8_t ecr_backup_; // so we can re-enable the correct number of electrodes
  // // when recovering from stop mode
  // uint8_t error_byte_;
  // bool running_;

  // int filtered_data_[ELECTRODE_COUNT];
  // int baseline_data_[ELECTRODE_COUNT];
  // uint16_t touch_data_;
  // uint16_t touch_data_previous_;
  // bool any_touched_flag_;

  // // writeRegister() and readRegister() manipulate registers on
  // // the MPR121 directly, whilst correctly stopping and
  // // restarting the MPR121 if necessary
  // void writeRegister(const uint8_t reg, const uint8_t value);
  // uint8_t readRegister(const uint8_t reg);

  // // applies a complete array of settings from a
  // // Settings variable useful if you want to do a bulk setup of the device
  // void applySettings(DeviceAddress device_address,
  //   const Settings & settings);

  // bool previouslyTouched(const uint8_t electrode);

};

#include "MPR121/MPR121Definitions.h"

#endif
