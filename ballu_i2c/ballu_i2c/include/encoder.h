#ifndef I2C_AM4096_H
#define I2C_AM4096_H

#include "i2c_device.h"

#define PERM_ADDR_ID  0
#define ADDR_ZERO_POS 1
#define ADDR_SIGN     1
#define ADDR_POS_REL 32
#define ADDR_POS_ABS 33
#define ADDR_MAGNET  34
#define ADDR_ID      48

class AM4096 : public I2CDevice { // AM4096
protected:
  const bool inRadian;
  float rangeMax;
  void _init(void) override;

public:
  AM4096(int i2c_dev, int bus_num, int16_t dev_id, bool wrapToPi=true, bool inRadian=true);
  ~AM4096();
  void reset(void) override;
  void update(void) override;
  void updateMagnetStatus(bool verbose=false);
  int ping(bool verbose=false) override;

  uint8_t chipID(void);
  uint16_t getOffset(bool verbose=false);
  uint32_t calibrateOffset(uint16_t offset, bool verbose=false);
  uint32_t calibrateOffset(bool verbose=false);
  uint8_t getRotationSign(bool verbose=false);
  uint32_t setRotationSign(bool positive, bool verbose=false);
  void updateId(unsigned new_id, float wait_sec=3.0, bool verbose=true);
  float wrap(float unwrapped);
  float unwrap(float wrapped);

  bool wrapToPi;

  float pos;       // reletive joint positions
  uint16_t rawPos; // raw joint values directly read from the sensor
  uint16_t valid;  // if the encoder value is valid;

  // float posAbs;
  // int16_t rawPosAbs;
  // int16_t validAbs;

  uint16_t rawMagnet;
  uint16_t magnetDist[2]; // [0]: far, [1]: close. 0 means normal.
};

#endif // I2C_AM4096_H
