#include <Arduino.h>
#include <MPR121.h>

#include "Constants.h"


MPR121 mpr121;

void setup()
{
  Serial.begin(constants::baud);

  mpr121.setWire(*constants::wire_ptr,
    constants::fast_mode);

  for (size_t device_index=0; device_index<constants::DEVICE_COUNT; ++device_index)
  {
    MPR121::DeviceAddress device_address = constants::device_addresses[device_index];
    mpr121.addDevice(device_address);
    mpr121.setupDevice(device_address);

    mpr121.setAllDeviceChannelsThresholds(device_address,
      constants::touch_threshold,
      constants::release_threshold);
    mpr121.setDebounce(device_address,
      constants::touch_debounce,
      constants::release_debounce);
    mpr121.setBaselineTracking(device_address,
      constants::baseline_tracking);
    mpr121.setChargeDischargeCurrent(device_address,
      constants::charge_discharge_current);
    mpr121.setChargeDischargeTime(device_address,
      constants::charge_discharge_time);
    mpr121.setFirstFilterIterations(device_address,
      constants::first_filter_iterations);
    mpr121.setSecondFilterIterations(device_address,
      constants::second_filter_iterations);
    mpr121.setSamplePeriod(device_address,
      constants::sample_period);

    mpr121.startChannels(device_address,
      constants::physical_channel_count,
      constants::proximity_mode);
  }
}

void loop()
{
  delay(constants::loop_delay);

  for (size_t device_index=0; device_index<constants::DEVICE_COUNT; ++device_index)
  {
    MPR121::DeviceAddress device_address = constants::device_addresses[device_index];

    Serial.print("device_index: ");
    Serial.println(device_index);

    if (!mpr121.communicating(device_address))
    {
      Serial.println("mpr121 device not commmunicating!\n");
      return;
    }

    uint8_t device_channel_count = mpr121.getDeviceChannelCount();
    Serial.print("device_channel_count: ");
    Serial.println(device_channel_count);
    uint8_t running_channel_count = mpr121.getRunningChannelCount(device_address);
    Serial.print("running_channel_count: ");
    Serial.println(running_channel_count);

    uint16_t touch_status = mpr121.getTouchStatus(device_address);
    Serial.print("touch_status: ");
    Serial.println(touch_status, BIN);
    if (mpr121.overCurrentDetected(touch_status))
    {
      Serial.println("Over current detected!\n");
      mpr121.startChannels(constants::physical_channel_count,
        constants::proximity_mode);
      return;
    }
    bool any_touched = mpr121.anyTouched(touch_status);
    Serial.print("any_touched: ");
    Serial.println(any_touched);
    bool device_channel_touched = mpr121.deviceChannelTouched(touch_status,
      constants::channel);
    Serial.print("device_channel_touched: ");
    Serial.println(device_channel_touched);

    uint16_t out_of_range_status = mpr121.getOutOfRangeStatus(device_address);
    Serial.print("out_of_range_status: ");
    Serial.println(out_of_range_status, BIN);

    bool channel_touched = mpr121.channelTouched(constants::channel);
    Serial.print("channel_touched: ");
    Serial.println(channel_touched);

    uint16_t channel_filtered_data = mpr121.getChannelFilteredData(constants::channel);
    Serial.print("channel_filtered_data: ");
    Serial.println(channel_filtered_data);

    uint16_t channel_baseline_data = mpr121.getChannelBaselineData(constants::channel);
    Serial.print("channel_baseline_data: ");
    Serial.println(channel_baseline_data);

    Serial.println();
  }
}
