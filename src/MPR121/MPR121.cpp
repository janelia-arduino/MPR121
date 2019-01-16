// ----------------------------------------------------------------------------
// MPR121.cpp
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#include "MPR121.h"

MPR121::MPR121()
{
  device_count_ = 0;
  touch_status_mask_ = TOUCH_STATUS_MASK_BASE;
}

bool MPR121::setupSingleDevice(TwoWire & wire,
  DeviceAddress device_address,
  bool fast_mode)
{
  setWire(Wire,fast_mode);
  addDevice(device_address);
  bool successfully_setup = setupAllDevices();
  return successfully_setup;
}

// proximity_mode:
// set number of channels to use to generate virtual "13th"
// proximity channel
// see http://cache.freescale.com/files/sensors/doc/app_note/AN3893.pdf
//
// N.B. - this is not related to general proximity detection or
// reading back continuous proximity data
// "13th channel" proximity modes
// N.B. this does not relate to normal proximity detection
// see http://cache.freescale.com/files/sensors/doc/app_note/AN3893.pdf
void MPR121::startChannels(uint8_t physical_channel_count,
  ProximityMode proximity_mode)
{
  startChannelsAllDevices(physical_channel_count,proximity_mode);
}

void MPR121::startAllChannels(ProximityMode proximity_mode)
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    startAllChannels(device_addresses_[device_index],proximity_mode);
  }
}

void MPR121::stopAllChannels()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    stopAllChannels(device_addresses_[device_index]);
  }
}

uint8_t MPR121::getChannelCount()
{
  return CHANNELS_PER_DEVICE * device_count_;
}

uint8_t MPR121::getRunningChannelCount()
{
  uint8_t running_channel_count = 0;
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    running_channel_count += getRunningChannelCount(device_addresses_[device_index]);
  }
  return running_channel_count;
}

void MPR121::setChannelThresholds(uint8_t channel,
  uint8_t touch_threshold,
  uint8_t release_threshold)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  uint8_t device_index = channelToDeviceIndex(channel);
  uint8_t device_channel = channelToDeviceChannel(channel);
  DeviceAddress device_address = device_addresses_[device_index];

  setDeviceChannelThresholds(device_address,
    device_channel,
    touch_threshold,
    release_threshold);
}

void MPR121::setAllChannelsThresholds(uint8_t touch_threshold,
  uint8_t release_threshold)
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    setAllDeviceChannelsThresholds(device_addresses_[device_index],
      touch_threshold,
      release_threshold);
  }
}

void MPR121::setWire(TwoWire & wire,
  bool fast_mode)
{
  wire_ptr_ = &wire;
  wire_ptr_->begin();
  if (fast_mode)
  {
    wire_ptr_->setClock(FAST_MODE_CLOCK_FREQUENCY);
  }

}

void MPR121::addDevice(DeviceAddress device_address)
{
  if (device_count_ >= DEVICE_COUNT_MAX)
  {
    return;
  }
  uint8_t device_index = device_count_++;
  device_addresses_[device_index] = device_address;
  proximity_modes_[device_index] = DISABLED;
}

bool MPR121::setupAllDevices()
{
  bool successfully_setup;
  bool all_successfully_setup = true;
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    successfully_setup = setup(device_addresses_[device_index]);
    all_successfully_setup = (all_successfully_setup && successfully_setup);
  }
  return all_successfully_setup;
}

void MPR121::startChannels(DeviceAddress device_address,
  uint8_t physical_channel_count,
  ProximityMode proximity_mode)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  proximity_modes_[device_index] = proximity_mode;

  clearOverCurrentFlag(device_address);

  ElectrodeConfiguration electrode_configuration;
  read(device_address,ECR_REGISTER_ADDRESS,electrode_configuration.uint8);
  if (physical_channel_count > PHYSICAL_CHANNELS_PER_DEVICE)
  {
    physical_channel_count = PHYSICAL_CHANNELS_PER_DEVICE;
  }
  electrode_configuration.fields.electrode_enable = physical_channel_count;
  electrode_configuration.fields.proximity_enable = proximity_mode;
  write(device_address,ECR_REGISTER_ADDRESS,electrode_configuration.uint8);
}

void MPR121::startChannelsAllDevices(uint8_t physical_channel_count,
  ProximityMode proximity_mode)
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    startChannels(device_addresses_[device_index],
      physical_channel_count,
      proximity_mode);
  }
}

void MPR121::startAllChannels(DeviceAddress device_address,
  ProximityMode proximity_mode)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  proximity_modes_[device_index] = proximity_mode;

  clearOverCurrentFlag(device_address);

  ElectrodeConfiguration electrode_configuration;
  read(device_address,ECR_REGISTER_ADDRESS,electrode_configuration.uint8);
  electrode_configuration.fields.electrode_enable = PHYSICAL_CHANNELS_PER_DEVICE;
  electrode_configuration.fields.proximity_enable = proximity_mode;
  write(device_address,ECR_REGISTER_ADDRESS,electrode_configuration.uint8);
}

void MPR121::stopAllChannels(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  ElectrodeConfiguration electrode_configuration;
  read(device_address,ECR_REGISTER_ADDRESS,electrode_configuration.uint8);
  electrode_configuration.fields.electrode_enable = 0;
  electrode_configuration.fields.proximity_enable = 0;
  write(device_address,ECR_REGISTER_ADDRESS,electrode_configuration.uint8);
}

uint8_t MPR121::getDeviceCount()
{
  return device_count_;
}

uint8_t MPR121::getDeviceChannelCount()
{
  return CHANNELS_PER_DEVICE;
}

uint8_t MPR121::getRunningChannelCount(DeviceAddress device_address)
{
  ElectrodeConfiguration electrode_configuration;
  read(device_address,ECR_REGISTER_ADDRESS,electrode_configuration.uint8);
  uint8_t running_channel_count = electrode_configuration.fields.electrode_enable;
  if (running_channel_count >= PHYSICAL_CHANNELS_PER_DEVICE)
  {
    running_channel_count = PHYSICAL_CHANNELS_PER_DEVICE;
  }
  Serial << "electrode_configuration.fields.proximity_enable: " << electrode_configuration.fields.proximity_enable << "\n";
  if (electrode_configuration.fields.proximity_enable)
  {
    running_channel_count += 1;
  }
  return running_channel_count;
}

void MPR121::setDeviceChannelThresholds(DeviceAddress device_address,
  uint8_t device_channel,
  uint8_t touch_threshold,
  uint8_t release_threshold)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  if (device_channel >= CHANNELS_PER_DEVICE)
  {
    return;
  }
  if (release_threshold > touch_threshold)
  {
    uint8_t threshold = touch_threshold;
    touch_threshold = release_threshold;
    release_threshold = threshold;
  }
  uint8_t register_address;
  pauseChannels(device_address);
  register_address = E0TTH_REGISTER_ADDRESS + device_channel * 2;
  write(device_address,register_address,touch_threshold);
  register_address = E0RTH_REGISTER_ADDRESS + device_channel * 2;
  write(device_address,register_address,release_threshold);
  resumeChannels(device_address);
}

void MPR121::setAllDeviceChannelsThresholds(DeviceAddress device_address,
  uint8_t touch_threshold,
  uint8_t release_threshold)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  if (release_threshold > touch_threshold)
  {
    uint8_t threshold = touch_threshold;
    touch_threshold = release_threshold;
    release_threshold = threshold;
  }
  uint8_t register_address;
  pauseChannels(device_address);
  for (uint8_t device_channel=0; device_channel<CHANNELS_PER_DEVICE; ++device_channel)
  {
    register_address = E0TTH_REGISTER_ADDRESS + device_channel * 2;
    write(device_address,register_address,touch_threshold);
    register_address = E0RTH_REGISTER_ADDRESS + device_channel * 2;
    write(device_address,register_address,release_threshold);
  }
  resumeChannels(device_address);
}

uint16_t MPR121::getTouchStatus(DeviceAddress device_address)
{
  uint16_t touch_status;
  read(device_address,TOUCH_STATUS_REGISTER_ADDRESS,touch_status);
  return touch_status;
}

bool MPR121::overCurrentDetected(uint16_t touch_status)
{
  return touch_status & OVER_CURRENT_REXT;
}

bool MPR121::anyTouched(uint16_t touch_status)
{
  return touch_status & touch_status_mask_;
}

uint8_t MPR121::getTouchCount(uint16_t touch_status)
{
  return touch_status & touch_status_mask_;
}

bool MPR121::deviceChannelTouched(uint16_t touch_status,
  uint8_t device_channel)
{
  return touch_status & (1 << device_channel);
}

// private

int MPR121::deviceAddressToDeviceIndex(DeviceAddress device_address)
{
  int device_index = DEVICE_INDEX_NONE;
  for (uint8_t index=0; index<device_count_; ++index)
  {
    if (device_address == device_addresses_[index])
    {
      device_index = index;
      break;
    }
  }
  return device_index;
}

bool MPR121::setup(DeviceAddress device_address)
{
  // soft reset
  write(device_address,SRST_REGISTER_ADDRESS,SRST_REGISTER_VALUE);
  delay(1);

  uint8_t register_data = 0;
  read(device_address,CDT_REGISTER_ADDRESS,register_data);

  if (register_data == CDT_REGISTER_DEFAULT)
  {
    applySettings(device_address,default_settings_);
    return SUCCESS;
  }
  else
  {
    return !SUCCESS;
  }
}

// Error MPR121::getError()
// {
//   // important - this resets the IRQ pin - as does any I2C comms

//   readRegister(OORSL_REGISTER_ADDRESS); // OOR registers - we may not have read them yet,
//   readRegister(OORSH_REGISTER_ADDRESS); // whereas the other errors should have been caught

//   // order of error precedence is determined in this logic block

//   if (!isInitialized())
//   {
//     return NOT_INITIALIZED; // this has its own checker function
//   }

//   if (error_byte_ & (1<<ADDRESS_UNKNOWN_BIT))
//   {
//     return ADDRESS_UNKNOWN;
//   }
//   else if (error_byte_ & (1<<READBACK_FAIL_BIT))
//   {
//     return READBACK_FAIL;
//   }
//   else if (error_byte_ & (1<<OVERCURRENT_FLAG_BIT))
//   {
//     return OVERCURRENT_FLAG;
//   }
//   else if (error_byte_ & (1<<OUT_OF_RANGE_BIT))
//   {
//     return OUT_OF_RANGE;
//   }
//   else
//   {
//     return NO_ERROR;
//   }
// }

// void MPR121::clearError()
// {
//   error_byte_ = 0;
// }

// void MPR121::updateTouchData()
// {
//   if (!isInitialized())
//   {
//     return;
//   }

//   any_touched_flag_ = false;

//   touch_data_previous_ = touch_data_;
//   touch_data_ = (uint16_t)readRegister(TSL_REGISTER_ADDRESS) + ((uint16_t)readRegister(TSH_REGISTER_ADDRESS)<<8);
// }

// bool MPR121::updateBaselineData()
// {
//   if (!isInitialized())
//   {
//     return !SUCCESS;
//   }

//   wire_ptr_->beginTransmission(address);
//   wire_ptr_->write(E0BV);   // set address register to read from the start of the
//   // baseline data
//   wire_ptr_->endTransmission(false); // repeated start

//   if (anyTouched())
//   {
//     any_touched_flag_ = true;
//   }

//   if (wire_ptr_->requestFrom(address,(uint8_t)ELECTRODE_COUNT) == ELECTRODE_COUNT)
//   {
//     for (size_t electrode=0; electrode<ELECTRODE_COUNT; ++electrode)
//     { // ELECTRODE_COUNT filtered values
//       if(anyTouched())
//       {
//         any_touched_flag_ = true;
//       }
//       baseline_data_[electrode] = wire_ptr_->read()<<2;
//     }
//     return SUCCCESS;
//   }
//   else
//   {
//     // if we don't get back all 26 values we requested, don't update the BVAL values
//     // and return !SUCCESS
//     return !SUCCESS;
//   }
// }

// bool MPR121::updateFilteredData()
// {
//   if (!isInitialized())
//   {
//     return !SUCCESS;
//   }

//   uint8_t LSB, MSB;

//   wire_ptr_->beginTransmission(address);
//   wire_ptr_->write(E0FDL); // set address register to read from the start of the
//   //filtered data
//   wire_ptr_->endTransmission(false); // repeated start

//   if (anyTouched())
//   {
//     any_touched_flag_ = true;
//   }

//   if (wire_ptr_->requestFrom(address,(uint8_t)26)==26)
//   {
//     for(size_t electrode=0; electrode<ELECTRODE_COUNT; ++electrode)
//     { // ELECTRODE_COUNT filtered values
//       if(anyTouched())
//       {
//         any_touched_flag_ = true;
//       }
//       LSB = wire_ptr_->read();
//       if(anyTouched())
//       {
//         any_touched_flag_ = true;
//       }
//       MSB = wire_ptr_->read();
//       filtered_data_[electrode] = ((MSB << 8) | LSB);
//     }
//     return SUCCESS;
//   }
//   else
//   {
//     // if we don't get back all 26 values we requested, don't update the FDAT values
//     // and return !SUCCESS
//     return !SUCCESS;
//   }
// }

// void MPR121::updateAll()
// {
//   updateTouchData();
//   updateBaselineData();
//   updateFilteredData();
// }

// bool MPR121::touched(uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return false; // avoid out of bounds behaviour
//   }

//   return (touch_data_>>electrode)&1;
// }

// uint8_t MPR121::getTouchCount()
// {
//   uint8_t touch_count = 0;

//   if (isInitialized())
//   {
//     for (size_t electrode=0; electrode<ELECTRODE_COUNT; ++electrode)
//     {
//       if (touched(electrode))
//       {
//         ++touch_count;
//       }
//     }
//   }

//   return touch_count;
// }

// int MPR121::getBaselineData(uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return(0xFFFF); // avoid out of bounds behaviour
//   }

//   return baseline_data_[electrode];
// }

// int MPR121::getFilteredData(uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return(0xFFFF); // avoid out of bounds behaviour
//   }

//   return filtered_data_[electrode];
// }

// bool MPR121::isNewTouch(uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return false; // avoid out of bounds behaviour
//   }
//   return ((previouslyTouched(electrode) == false) && (touched(electrode) == true));
// }

// bool MPR121::isNewRelease(uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return false; // avoid out of bounds behaviour
//   }
//   return ((previouslyTouched(electrode) == true) && (touched(electrode) == false));
// }

// void MPR121::setInterruptPin(const int pin)
// {
//   interrupt_pin_ = pin;

//   if (pin >= 0))
//   {
//     ::pinMode(pin,INPUT_PULLUP);
//   }
// }

// void MPR121::setProximityMode(const ProximityMode mode)
// {

//   if (!isInitialized())
//   {
//     return;
//   }

//   bool was_running = running;

//   if (was_running)
//   {
//     stop();
//   }

//   switch(mode)
//   {
//     case DISABLED:
//     {
//       ecr_backup_ &= ~(3<<4);  // ELEPROX[0:1] = 00
//       break;
//     }
//     case PROX0_1:
//     {
//       ecr_backup_ |=  (1<<4);  // ELEPROX[0:1] = 01
//       ecr_backup_ &= ~(1<<5);
//       break;
//     }
//     case PROX0_3:
//     {
//       ecr_backup_ &= ~(1<<4);  // ELEPROX[0:1] = 10
//       ecr_backup_ |=  (1<<5);
//       break;
//     }
//     case PROX0_11:
//     {
//       ecr_backup_ |=  (3<<4);  // ELEPROX[0:1] = 11
//       break;
//     }
//   }

//   if (was_running)
//   {
//     run();
//   }
// }

// void MPR121::setDigitalPinCount(uint8_t pin_count)
// {
//   if(!isInitialized()) return;
//   bool was_running = running;

//   if (pin_count > DIGITAL_PIN_COUNT_MAX)
//   {
//     pin_count = DIGITAL_PIN_COUNT_MAX;
//   }

//   if (was_running)
//   {
//     stop(); // have to stop to change ECR
//   }
//   ecr_backup_ = (0x0F & ((ELECTRODE_COUNT - 1) - pin_count)) | (ecr_backup_ & 0xF0);
//   if (was_running)
//   {
//     run();
//   }

// }

// void MPR121::pinMode(uint8_t electrode, const PinMode mode)
// {

//   // only valid for ELE4..ELE11
//   if(electrode<4 || electrode >11 || !isInitialized()) return;

//   // LED0..LED7
//   uint8_t bitmask = 1<<(electrode-4);

//   switch(mode){
//     case INPUT_PU:
//       // EN = 1
//       // DIR = 0
//       // CTL0 = 1
//       // CTL1 = 1
//       writeRegister(EN, readRegister(EN) | bitmask);
//       writeRegister(DIR, readRegister(DIR) & ~bitmask);
//       writeRegister(CTL0, readRegister(CTL0) | bitmask);
//       writeRegister(CTL1, readRegister(CTL1) | bitmask);
//       break;
//     case INPUT_PD:
//       // EN = 1
//       // DIR = 0
//       // CTL0 = 1
//       // CTL1 = 0
//       writeRegister(EN, readRegister(EN) | bitmask);
//       writeRegister(DIR, readRegister(DIR) & ~bitmask);
//       writeRegister(CTL0, readRegister(CTL0) | bitmask);
//       writeRegister(CTL1, readRegister(CTL1) & ~bitmask);
//       break;
//     case OUTPUT_HS:
//       // EN = 1
//       // DIR = 1
//       // CTL0 = 1
//       // CTL1 = 1
//       writeRegister(EN, readRegister(EN) | bitmask);
//       writeRegister(DIR, readRegister(DIR) | bitmask);
//       writeRegister(CTL0, readRegister(CTL0) | bitmask);
//       writeRegister(CTL1, readRegister(CTL1) | bitmask);
//       break;
//     case OUTPUT_LS:
//       // EN = 1
//       // DIR = 1
//       // CTL0 = 1
//       // CTL1 = 0
//       writeRegister(EN, readRegister(EN) | bitmask);
//       writeRegister(DIR, readRegister(DIR) | bitmask);
//       writeRegister(CTL0, readRegister(CTL0) | bitmask);
//       writeRegister(CTL1, readRegister(CTL1) & ~bitmask);
//       break;
//   }
// }

// void MPR121::pinMode(uint8_t electrode, int mode)
// {
//   if(!isInitialized()) return;

//   // this is to catch the fact that Arduino prefers its definition of INPUT
//   // and OUTPUT to ours...

//   uint8_t bitmask = 1<<(electrode-4);

//   if(mode == OUTPUT){
//     // EN = 1
//     // DIR = 1
//     // CTL0 = 0
//     // CTL1 = 0
//     writeRegister(EN, readRegister(EN) | bitmask);
//     writeRegister(DIR, readRegister(DIR) | bitmask);
//     writeRegister(CTL0, readRegister(CTL0) & ~bitmask);
//     writeRegister(CTL1, readRegister(CTL1) & ~bitmask);

//   } else if(mode == INPUT){
//     // EN = 1
//     // DIR = 0
//     // CTL0 = 0
//     // CTL1 = 0
//     writeRegister(EN, readRegister(EN) | bitmask);
//     writeRegister(DIR, readRegister(DIR) & ~bitmask);
//     writeRegister(CTL0, readRegister(CTL0) & ~bitmask);
//     writeRegister(CTL1, readRegister(CTL1) & ~bitmask);
//   } else {
//     return; // anything that isn't a 1 or 0 is invalid
//   }
// }

// void MPR121::digitalWrite(uint8_t electrode, uint8_t val)
// {

//   // avoid out of bounds behaviour

//   if(electrode<4 || electrode>11 || !isInitialized()) return;

//   if(val){
//     writeRegister(SET, 1<<(electrode-4));
//   } else {
//     writeRegister(CLR, 1<<(electrode-4));
//   }
// }

// void MPR121::digitalToggle(uint8_t electrode)
// {

//   // avoid out of bounds behaviour

//   if(electrode<4 || electrode>11 || !isInitialized()) return;

//   writeRegister(TOG, 1<<(electrode-4));
// }

// bool MPR121::digitalRead(uint8_t electrode)
// {

//   // avoid out of bounds behaviour

//   if(electrode<4 || electrode>11 || !isInitialized()) return !SUCCESS;

//   return(((readRegister(DAT)>>(electrode-4))&1)==1);
// }

// void MPR121::analogWrite(uint8_t electrode, uint8_t value)
// {
//   // LED output 5 (ELE9) and output 6 (ELE10) have a PWM bug
//   // https://community.nxp.com/thread/305474

//   // avoid out of bounds behaviour

//   if(electrode<4 || electrode>11 || !isInitialized()) return;

//   uint8_t shiftedVal = value>>4;

//   if(shiftedVal > 0){
//     writeRegister(SET, 1<<(electrode-4)); // normal PWM operation
//   } else {
//     // this make a 0 PWM setting turn off the output
//     writeRegister(CLR, 1<<(electrode-4));
//   }

//   uint8_t scratch;

//   switch(electrode-4){

//     case 0:
//       scratch = readRegister(PWM0);
//       writeRegister(PWM0, (shiftedVal & 0x0F) | (scratch & 0xF0));
//       break;
//     case 1:
//       scratch = readRegister(PWM0);
//       writeRegister(PWM0, ((shiftedVal & 0x0F)<<4) | (scratch & 0x0F));
//       break;
//     case 2:
//       scratch = readRegister(PWM1);
//       writeRegister(PWM1, (shiftedVal & 0x0F) | (scratch & 0xF0));
//       break;
//     case 3:
//       scratch = readRegister(PWM1);
//       writeRegister(PWM1, ((shiftedVal & 0x0F)<<4) | (scratch & 0x0F));
//       break;
//     case 4:
//       scratch = readRegister(PWM2);
//       writeRegister(PWM2, (shiftedVal & 0x0F) | (scratch & 0xF0));
//       break;
//     case 5:
//       scratch = readRegister(PWM2);
//       writeRegister(PWM2, ((shiftedVal & 0x0F)<<4) | (scratch & 0x0F));
//       break;
//     case 6:
//       scratch = readRegister(PWM3);
//       writeRegister(PWM3, (shiftedVal & 0x0F) | (scratch & 0xF0));
//       break;
//     case 7:
//       scratch = readRegister(PWM3);
//       writeRegister(PWM3, ((shiftedVal & 0x0F)<<4) | (scratch & 0x0F));
//       break;
//   }
// }

// void MPR121::setSamplePeriod(SamplePeriod period)
// {
//   uint8_t scratch;

//   scratch = readRegister(CDT);
//   writeRegister(CDT, (scratch & 0xF8) | (period & 0x07));
// }

// void MPR121::writeRegister(uint8_t reg, uint8_t value)
// {

//   bool was_running = false;;

//   if (reg == ECR)
//   { // if we are modding ECR, update our internal running status
//     if (value&0x3F)
//     {
//       running_ = true;
//     }
//     else
//     {
//       running_ = false;
//     }
//   }
//   else if (reg < CTL0)
//   {
//     was_running = running_;
//     if (was_running)
//     {
//       stop();  // we should ALWAYS be in stop mode for this
//     }
//     // unless modding ECR or GPIO / LED register
//   }

//   wire_ptr_->beginTransmission(address_);
//   wire_ptr_->write(reg);
//   wire_ptr_->write(value);
//   if (wire_ptr_->endTransmission()!=0)
//   {
//     error_byte_ |= 1<<ADDRESS_UNKNOWN_BIT; // set address unknown bit
//   }
//   else
//   {
//     error_byte_ &= ~(1<<ADDRESS_UNKNOWN_BIT);
//   }

//   if (was_running)
//   {
//     run();   // restore run mode if necessary
//   }
// }

// uint8_t MPR121::readRegister(uint8_t reg)
// {
//   uint8_t value;

//   wire_ptr_->beginTransmission(address);
//   wire_ptr_->write(reg); // set address to read from our requested register
//   wire_ptr_->endTransmission(false); // repeated start
//   wire_ptr_->requestFrom(address,(uint8_t)1);  // just a single byte
//   if (wire_ptr_->endTransmission()!=0)
//   {
//     error_byte_ |= 1<<ADDRESS_UNKNOWN_BIT;
//   }
//   else
//   {
//     error_byte_ &= ~(1<<ADDRESS_UNKNOWN_BIT);
//   }
//   value = wire_ptr_->read();
//   // auto update errors for registers with error data
//   if (reg == TSH_REGISTER_ADDRESS && ((value&0x80)!=0))
//   {
//     error_byte_ |= 1<<OVERCURRENT_FLAG_BIT;
//   }
//   else
//   {
//     error_byte_ &= ~(1<<OVERCURRENT_FLAG_BIT);
//   }
//   if ((reg == OORSL_REGISTER_ADDRESS || reg == OORSH_REGISTER_ADDRESS) && (value!=0))
//   {
//     error_byte_ |= 1<<OUT_OF_RANGE_BIT;
//   }
//   else
//   {
//     error_byte_ &= ~(1<<OUT_OF_RANGE_BIT);
//   }
//   return value;
// }

void MPR121::pauseChannels(DeviceAddress device_address)
{
  read(device_address,ECR_REGISTER_ADDRESS,electrode_configuration_backup_.uint8);
  stopAllChannels(device_address);
}

void MPR121::resumeChannels(DeviceAddress device_address)
{
  write(device_address,ECR_REGISTER_ADDRESS,electrode_configuration_backup_.uint8);
}

uint8_t MPR121::channelToDeviceIndex(uint8_t channel)
{
  return channel / CHANNELS_PER_DEVICE;
}

uint8_t MPR121::channelToDeviceChannel(uint8_t channel)
{
  return channel % CHANNELS_PER_DEVICE;
}

void MPR121::applySettings(DeviceAddress device_address,
  const Settings & settings)
{
  stopAllChannels(device_address);

  write(device_address,MHDR_REGISTER_ADDRESS,settings.MHDR);
  write(device_address,NHDR_REGISTER_ADDRESS,settings.NHDR);
  write(device_address,NCLR_REGISTER_ADDRESS,settings.NCLR);
  write(device_address,FDLR_REGISTER_ADDRESS,settings.FDLR);
  write(device_address,MHDF_REGISTER_ADDRESS,settings.MHDF);
  write(device_address,NHDF_REGISTER_ADDRESS,settings.NHDF);
  write(device_address,NCLF_REGISTER_ADDRESS,settings.NCLF);
  write(device_address,FDLF_REGISTER_ADDRESS,settings.FDLF);
  write(device_address,NHDT_REGISTER_ADDRESS,settings.NHDT);
  write(device_address,NCLT_REGISTER_ADDRESS,settings.NCLT);
  write(device_address,FDLT_REGISTER_ADDRESS,settings.FDLT);
  write(device_address,MHDPROXR_REGISTER_ADDRESS,settings.MHDPROXR);
  write(device_address,NHDPROXR_REGISTER_ADDRESS,settings.NHDPROXR);
  write(device_address,NCLPROXR_REGISTER_ADDRESS,settings.NCLPROXR);
  write(device_address,FDLPROXR_REGISTER_ADDRESS,settings.FDLPROXR);
  write(device_address,MHDPROXF_REGISTER_ADDRESS,settings.MHDPROXF);
  write(device_address,NHDPROXF_REGISTER_ADDRESS,settings.NHDPROXF);
  write(device_address,NCLPROXF_REGISTER_ADDRESS,settings.NCLPROXF);
  write(device_address,FDLPROXF_REGISTER_ADDRESS,settings.FDLPROXF);
  write(device_address,NHDPROXT_REGISTER_ADDRESS,settings.NHDPROXT);
  write(device_address,NCLPROXT_REGISTER_ADDRESS,settings.NCLPROXT);
  write(device_address,FDLPROXT_REGISTER_ADDRESS,settings.FDLPROXT);
  write(device_address,DTR_REGISTER_ADDRESS,settings.DTR);
  write(device_address,CDC_REGISTER_ADDRESS,settings.CDC);
  write(device_address,CDT_REGISTER_ADDRESS,settings.CDT);
  write(device_address,ECR_REGISTER_ADDRESS,settings.ECR);
  write(device_address,ACCR0_REGISTER_ADDRESS,settings.ACCR0);
  write(device_address,ACCR1_REGISTER_ADDRESS,settings.ACCR1);
  write(device_address,USL_REGISTER_ADDRESS,settings.USL);
  write(device_address,LSL_REGISTER_ADDRESS,settings.LSL);
  write(device_address,TL_REGISTER_ADDRESS,settings.TL);

  setAllDeviceChannelsThresholds(device_address,
    settings.touch_threshold,
    settings.release_threshold);
  // setInterruptPin(settings.INTERRUPT);
}

void MPR121::clearOverCurrentFlag(DeviceAddress device_address)
{
  write(device_address,TOUCH_STATUS_REGISTER_ADDRESS,OVER_CURRENT_REXT);
}

// bool MPR121::previouslyTouched(uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return false; // avoid out of bounds behaviour
//   }

//   return ((touch_data_previous_>>electrode)&1);
// }
