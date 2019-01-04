// ----------------------------------------------------------------------------
// MPR121.cpp
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// Jim Lindblom
// Stefan Dzisiewski-Smith
// Peter Krige
// Adafruit <info@adafruit.com>
// ----------------------------------------------------------------------------
#include "MPR121.h"

MPR121::MPR121()
{
  device_count_ = 0;
  // wire_ptr_ = &Wire;
  // address_ = 0x5A;
  // ecr_backup_ = 0x00;
  // error_byte_ = 1<<NOT_INITIALIZED_BIT; // initially, we're not initialised
  // running_ = false;
  // touch_data_ = 0;
  // touch_data_previous_ = 0;
  // any_touched_flag_ = false;
}

void MPR121::setupSingleDevice(TwoWire & wire,
  DeviceAddress device_address,
  bool fast_mode)
{
  setWire(Wire,fast_mode);
  addDevice(device_address);
  resetAllDevices();
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
  device_addresses_[device_count_++] = device_address;
}

void MPR121::resetAllDevices()
{
  for (uint8_t device_index=0; device_index<device_count_; ++device_index)
  {
    reset(device_addresses_[device_index]);
  }
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

bool MPR121::reset(DeviceAddress device_address)
{
  // return SUCCESS if we successfully reset a device at the
  // address we are expecting

  // AFE2 is one of the few registers that defaults to a non-zero value -
  // checking it is sensible as reading back an incorrect value implies
  // something went wrong - we also check TSH bit 7 to see if we have an
  // overcurrent flag set

  // soft reset
  write(device_address,SRST_REGISTER_ADDRESS,SRST_REGISTER_VALUE);
  delay(1);

  uint8_t register_data = 0;
  read(device_address,CDT_REGISTER_ADDRESS,register_data);

  Serial << "cdt_register_data: " << _HEX(register_data) << "\n";

  if (register_data == CDT_REGISTER_DEFAULT)
  {
    Serial << "success!!\n";
    return SUCCESS;
  }
  else
  {
    Serial << "fail!!\n";
    return !SUCCESS;
  }

  // if (readRegister(AFE2)!=0x24)
  // {
  //   error_byte_ |= 1<<READBACK_FAIL_BIT;
  // }
  // else
  // {
  //   error_byte_ &= ~(1<<READBACK_FAIL_BIT);
  // }

  // if ((readRegister(TSH)&0x80)!=0)
  // {
  //   error_byte_ |= 1<<OVERCURRENT_FLAG_BIT;
  // }
  // else
  // {
  //   error_byte_ &= ~(1<<OVERCURRENT_FLAG_BIT);
  // }

  // if (getError()==NOT_INITIALIZED || getError()==NO_ERROR)
  // { // if our only error is that we are not initialized...
  //   return SUCCESS;
  // }
  // else
  // {
  //   return !SUCCESS;
  // }
}

// bool MPR121::begin()
// {
//   wire_ptr_->begin();

//   error_byte_ &= ~(1<<NOT_INITIALIZED_BIT); // clear NOT_INITIALIZED error flag

//   if(reset())
//   {
//     applySettings(default_settings_);
//     return SUCCESS;
//   }
//   return !SUCCESS;
// }

// Error MPR121::getError()
// {
//   // important - this resets the IRQ pin - as does any I2C comms

//   readRegister(OORSL); // OOR registers - we may not have read them yet,
//   readRegister(OORSH); // whereas the other errors should have been caught

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
//   touch_data_ = (uint16_t)readRegister(TSL) + ((uint16_t)readRegister(TSH)<<8);
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

// bool MPR121::anyTouched()
// {
//   return any_touched_flag_;
// }

// bool MPR121::touched(const uint8_t electrode)
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

// int MPR121::getBaselineData(const uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return(0xFFFF); // avoid out of bounds behaviour
//   }

//   return baseline_data_[electrode];
// }

// int MPR121::getFilteredData(const uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return(0xFFFF); // avoid out of bounds behaviour
//   }

//   return filtered_data_[electrode];
// }

// bool MPR121::isNewTouch(const uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return false; // avoid out of bounds behaviour
//   }
//   return ((previouslyTouched(electrode) == false) && (touched(electrode) == true));
// }

// bool MPR121::isNewRelease(const uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return false; // avoid out of bounds behaviour
//   }
//   return ((previouslyTouched(electrode) == true) && (touched(electrode) == false));
// }

// void MPR121::setTouchThreshold(const uint8_t threshold)
// {
//   if (!isInitialized())
//   {
//     return;
//   }

//   bool was_running = running_;

//   if (was_running)
//   {
//     stop();  // can only change thresholds when not running
//   }
//   // checking here avoids multiple stop() / run()
//   // calls

//   for (size_t electrode=0; electrode<ELECTRODE_COUNT; ++electrode)
//   {
//     setTouchThreshold(electrode, threshold);
//   }

//   if (was_running)
//   {
//     run();
//   }
// }

// void MPR121::setTouchThreshold(const uint8_t electrode, const uint8_t threshold)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return; // avoid out of bounds behaviour
//   }

//   // this relies on the internal register map of the MPR121
//   writeRegister(E0TTH + (electrode<<1), threshold);
// }

// void MPR121::setReleaseThreshold(uint8_t threshold)
// {
//   if (!isInitialized())
//   {
//     return;
//   }

//   bool was_running = running;

//   if(was_running)
//   {
//     stop();  // can only change thresholds when not running
//   }
//   // checking here avoids multiple stop / starts

//   for (size_t electrode=0; electrode<ELECTRODE_COUNT; ++electrode)
//   {
//     setReleaseThreshold(electrode,threshold);
//   }

//   if (was_running)
//   {
//     run();
//   }
// }

// void MPR121::setReleaseThreshold(uint8_t electrode, uint8_t threshold)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return; // avoid out of bounds behaviour
//   }

//   // this relies on the internal register map of the MPR121
//   writeRegister(E0RTH + (electrode<<1), threshold);
// }

// uint8_t MPR121::getTouchThreshold(uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return(0xFF); // avoid out of bounds behaviour
//   }
//   return readRegister(E0TTH+(electrode<<1)); // "255" issue is in here somewhere
// }
// uint8_t MPR121::getReleaseThreshold(uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return(0xFF); // avoid out of bounds behaviour
//   }
//   return readRegister(E0RTH+(electrode<<1)); // "255" issue is in here somewhere
// }

// void MPR121::goSlow()
// {
//   wire_ptr_->setClock(100000L); // set I2C clock to 100kHz
// }

// void MPR121::goFast()
// {
//   wire_ptr_->setClock(400000L); // set I2C clock to 400kHz
// }

// void MPR121::run()
// {
//   if (!isInitialized())
//   {
//     return;
//   }
//   writeRegister(ECR,ecr_backup_); // restore backup to return to run mode
// }

// void MPR121::stop()
// {
//   if(!isInitialized())
//   {
//     return;
//   }
//   ecr_backup_ = readRegister(ECR);  // backup ECR to restore when we enter run
//   writeRegister(ECR, ecr_backup_ & 0xC0); // turn off all electrodes to stop
// }

// bool MPR121::isRunning()
// {
//   return running_;
// }

// bool MPR121::isInitialized()
// {
//   return !(error_byte_ & (1<<NOT_INITIALIZED_BIT));
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

// void MPR121::setDigitalPinCount(const uint8_t pin_count)
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

// void MPR121::pinMode(const uint8_t electrode, const PinMode mode)
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

//   scratch = readRegister(AFE2);
//   writeRegister(AFE2, (scratch & 0xF8) | (period & 0x07));
// }

// void MPR121::writeRegister(const uint8_t reg, const uint8_t value)
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

// uint8_t MPR121::readRegister(const uint8_t reg)
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
//   if (reg == TSH && ((value&0x80)!=0))
//   {
//     error_byte_ |= 1<<OVERCURRENT_FLAG_BIT;
//   }
//   else
//   {
//     error_byte_ &= ~(1<<OVERCURRENT_FLAG_BIT);
//   }
//   if ((reg == OORSL || reg == OORSH) && (value!=0))
//   {
//     error_byte_ |= 1<<OUT_OF_RANGE_BIT;
//   }
//   else
//   {
//     error_byte_ &= ~(1<<OUT_OF_RANGE_BIT);
//   }
//   return value;
// }

// void MPR121::applySettings(const Settings & settings)
// {
//   bool was_running = running;
//   if(was_running)
//   {
//     stop();  // can't change most regs when running - checking
//   }
//   // here avoids multiple stop() / run() calls

//   writeRegister(MHDR,settings->MHDR);
//   writeRegister(NHDR,settings->NHDR);
//   writeRegister(NCLR,settings->NCLR);
//   writeRegister(FDLR,settings->FDLR);
//   writeRegister(MHDF,settings->MHDF);
//   writeRegister(NHDF,settings->NHDF);
//   writeRegister(NCLF,settings->NCLF);
//   writeRegister(FDLF,settings->FDLF);
//   writeRegister(NHDT,settings->NHDT);
//   writeRegister(NCLT,settings->NCLT);
//   writeRegister(FDLT,settings->FDLT);
//   writeRegister(MHDPROXR,settings->MHDPROXR);
//   writeRegister(NHDPROXR,settings->NHDPROXR);
//   writeRegister(NCLPROXR,settings->NCLPROXR);
//   writeRegister(FDLPROXR,settings->FDLPROXR);
//   writeRegister(MHDPROXF,settings->MHDPROXF);
//   writeRegister(NHDPROXF,settings->NHDPROXF);
//   writeRegister(NCLPROXF,settings->NCLPROXF);
//   writeRegister(FDLPROXF,settings->FDLPROXF);
//   writeRegister(NHDPROXT,settings->NHDPROXT);
//   writeRegister(NCLPROXT,settings->NCLPROXT);
//   writeRegister(FDLPROXT,settings->FDLPROXT);
//   writeRegister(DTR, settings->DTR);
//   writeRegister(AFE1, settings->AFE1);
//   writeRegister(AFE2, settings->AFE2);
//   writeRegister(ACCR0, settings->ACCR0);
//   writeRegister(ACCR1, settings->ACCR1);
//   writeRegister(USL, settings->USL);
//   writeRegister(LSL, settings->LSL);
//   writeRegister(TL, settings->TL);

//   writeRegister(ECR, settings->ECR);

//   error_byte_ &= ~(1<<NOT_INITIALIZED_BIT); // clear not inited error as we have just inited!
//   setTouchThreshold(settings->TTHRESH);
//   setReleaseThreshold(settings->RTHRESH);
//   setInterruptPin(settings->INTERRUPT);

//   if(was_running)
//   {
//     run();
//   }
// }

// bool MPR121::previouslyTouched(const uint8_t electrode)
// {
//   if ((electrode >= ELECTRODE_COUNT) || !isInitialized())
//   {
//     return false; // avoid out of bounds behaviour
//   }

//   return ((touch_data_previous_>>electrode)&1);
// }
