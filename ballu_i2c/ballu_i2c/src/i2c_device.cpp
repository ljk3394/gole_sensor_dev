#include <iostream>
#include <unistd.h>
#include <string>
#include <sstream>
#include <algorithm>
#include <pigpiod_if2.h>

#include "i2c_device.h"

// PI_BAD_HANDLE: -25
// PI_BAD_PARAM: -81
// PI_I2C_READ_FAILED: -83

I2CDevice::I2CDevice(int i2c_dev, unsigned bus_num, unsigned dev_id)
    : i2c_dev(i2c_dev), bus_num(bus_num), id(dev_id) { _connect(); }
I2CDevice::~I2CDevice() { _close(); }
void I2CDevice::_connect() { i2c_handle = i2c_open(i2c_dev, bus_num, id, 0); }
void I2CDevice::_close() { i2c_close(i2c_dev, i2c_handle); }

int32_t I2CDevice::_readDev(unsigned count, uint8_t* buf) { // unlike other sister functions, this function will not do conversion for Edianness
  if(count == 1) {
    return i2c_read_byte(i2c_dev, i2c_handle);
  }

  if (buf == nullptr) buf = buffer;
  return i2c_read_device(i2c_dev, i2c_handle, (char *)buf, count);
}

int32_t I2CDevice::_writeDev(unsigned count, uint32_t val, uint8_t* buf) { // unlike other sister functions, this function will not do conversion for Edianness
  if(count == 1) return i2c_write_byte(i2c_dev, i2c_handle, val);

  if(buf == nullptr) buf = buffer;
  
  int ret = i2c_write_device(i2c_dev, i2c_handle, (char *)buf, count);
  if(ret == 0) {
    return count;
  } else {
    return ret; // error code
  }
}

uint32_t I2CDevice::_read(uint8_t i2c_reg, unsigned count, uint8_t* buf) {
  if(count == 1) { // 8 bit
    return 0xFF & i2c_read_byte_data(i2c_dev, i2c_handle, i2c_reg);
  } else if(count == 2) { // 16 bit
    int value = i2c_read_word_data(i2c_dev, i2c_handle, i2c_reg);
    return ((value & 0x00FF) << 8) + ((value & 0xFF00) >> 8);
  } else if(count == 4) {
    uint32_t res=0;
    i2c_read_i2c_block_data(i2c_dev, i2c_handle, i2c_reg, (char *)buffer, count); // returns number of bytes read
    for(size_t idx=0; idx < count; ++idx) {
      res = res << 8;
      res = res + (buffer[idx] & 0x00FF);
    }
    return res;
  } else { // up to 32 bytes
    int num_bytes_read = i2c_read_i2c_block_data(i2c_dev, i2c_handle, i2c_reg, (char *)buffer, count); // returns number of bytes read

    while(count-- > 0) { *(buf++) = *(buffer + count); } // flip
    return num_bytes_read;
  }
}

uint32_t I2CDevice::_read(uint8_t i2c_reg, unsigned count, uint8_t shift, uint8_t bits) {
  uint32_t raw = _read(i2c_reg, count);
  raw >>= shift;
  return raw & ((1 << (bits)) - 1);
}

uint32_t I2CDevice::_write(uint8_t i2c_reg, unsigned count, uint32_t val, uint8_t* buf) {
  int res = 0; // return error code
  if(count == 1) {
    res = i2c_write_byte_data(i2c_dev, i2c_handle, i2c_reg, 0xFF & val);
  } else if(count == 2) {
    val = ((val & 0x00FF) << 8) + ((val & 0xFF00) >> 8);
    res = i2c_write_word_data(i2c_dev, i2c_handle, i2c_reg, val);
  } else if(count == 4) {
    buffer[0] = (val >> 24) & 0xFF;
    buffer[1] = (val >> 16) & 0xFF;
    buffer[2] = (val >>  8) & 0xFF;
    buffer[3] = (val      ) & 0xFF;
    res = i2c_write_i2c_block_data(i2c_dev, i2c_handle, i2c_reg, (char *)buffer, count);
  } else {
    for(size_t idx = 0; idx < count; ++idx) { buffer[idx] = *(buf + (count-idx-1)); } 
    
    // int _cnt = count;
    // while(_cnt -- > 0) {*(buffer++) = *(buf + _cnt);}
    res = i2c_write_i2c_block_data(i2c_dev, i2c_handle, i2c_reg, (char *)buffer, count);
    
  }
  return res;
}

uint32_t I2CDevice::_write(uint8_t i2c_reg, unsigned count, uint32_t val, uint8_t shift, uint8_t bits) {
  int reg_val = _read(i2c_reg, count);

  // mask off the data before writing
  int mask = (1 << (bits)) - 1;
  val &= mask;

  mask <<= shift;
  reg_val &= ~mask;          // remove the current data at that spot
  reg_val |= val << shift;   // and add in the new data

  return _write(i2c_reg, count, reg_val);
}

uint32_t I2CDevice::_convertFromReadBuffer(uint8_t* buf, unsigned count) {
  uint32_t value = 0;
  for(size_t i=0; i < count; i++) { // LSBFIRST
    value <<= 8;
    value |= buf[count - i - 1];
  }
  return value;
}

void I2CDevice::_convertToWriteBuffer(uint8_t* buf, unsigned count, int value) {
  for(size_t i=0; i < count; i++) { // LSBFIRST
    buf[i] = value && 0xFF;
    value >>= 8;
  }
}

const char* I2CDevice::getI2CErrorMessage(int errorNum) {
    return pigpio_error(errorNum);
}

/** Utility Functions **/
string dec2hex(int dec) {
  stringstream ss;
  ss << hex << dec;
  string hex = string(ss.str());
  transform(hex.begin(), hex.end(), hex.begin(), ::toupper);
  return "0x" + hex;
}
