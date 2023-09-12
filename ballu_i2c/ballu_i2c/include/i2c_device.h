#ifndef I2C_DEVICE_H
#define I2C_DEVICE_H

#define BUFFER_SIZE 32

using namespace std;

class I2CDevice {
protected:
  const int i2c_dev;
  const uint8_t bus_num;
  uint8_t buffer[BUFFER_SIZE];

  void _connect();
  void _close();

  virtual void _init(void) {}; // = 0
  int32_t _readDev(unsigned count, uint8_t* buf=nullptr);
  int32_t _writeDev(unsigned count, uint32_t val, uint8_t* buf=nullptr);
  
  uint32_t _read(uint8_t i2c_reg, unsigned count, uint8_t* buf=nullptr);
  uint32_t _read(uint8_t i2c_reg, unsigned count, uint8_t shift, uint8_t bits);
  uint32_t _write(uint8_t i2c_reg, unsigned count, uint32_t val=0, uint8_t* buf=nullptr);
  uint32_t _write(uint8_t i2c_reg, unsigned count, uint32_t val, uint8_t shift, uint8_t bits);
  uint32_t _convertFromReadBuffer(uint8_t* buf, unsigned count);
  void _convertToWriteBuffer(uint8_t* buf, unsigned count, int value);

public:
  unsigned i2c_handle;
  unsigned id;
  I2CDevice(int i2c_dev, unsigned bus_num, unsigned dev_id); // = delete for abstract
  ~I2CDevice();

  virtual void update(void) {}; // (= 0 purely) abstract function
  virtual void reset(void) {};
  virtual int ping(bool verbose=false) = 0;
  const char* getI2CErrorMessage(int errorNum);
};

// utilities
string dec2hex(int dec);

#endif // I2C_DEVICE_H
