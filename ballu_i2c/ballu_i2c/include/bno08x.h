#ifndef _I2C_BNO08X_H
#define _I2C_BNO08X_H

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#include "i2c_device.h"

#define BNO08x_I2CADDR_DEFAULT 0x4A ///< The default I2C address

/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS 8 ///< Activity code for being on stairs
#define PAC_OPTION_COUNT                                                       \
  9 ///< The number of current options for the activity classifier

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the BNO08x 9-DOF Orientation IMU Fusion Breakout
 */
class BNO08X: public I2CDevice  {
public:
  BNO08X(int i2c_dev, int bus_num, int16_t dev_id, long period=5000, int8_t reset_pin=-1);
  BNO08X(int i2c_dev, int bus_num, int16_t dev_id, long periodAcc, long periodGyro, long periodOri, long periodGravity, long periodMag, int8_t reset_pin=-1);
  ~BNO08X();
  
  int32_t read(unsigned count, uint8_t* buf=nullptr);
  int32_t write(unsigned count, uint32_t val, uint8_t* buf=nullptr);

  void update(void) override;
  int ping(bool verbose=false) override;

  void reset(void) override;
  bool wasReset(void);
  bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us=10000);
  bool enableReports(void);

  sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor

  // report intervals in us
  long periodOri     = 5000;
  long periodAcc     = 5000; 
  long periodGyro    = 5000;
  long periodGravity = 5000;
  long periodMag     = 5000;

  // sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV; long reportIntervalUs = 2000; // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  // sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV; long reportIntervalUs = 5000; // Top frequency is about 250Hz but this report is more accurate

  uint8_t quality; // 0(poor) - 3(great)
  float accuracy;  // Accuracy estimate for rotation vector (or quaternion) [radians]
  float acc[3], gyro[3], qt[4], g[3], mag[3]; 


protected:
//   virtual bool _init(int32_t sensor_id);
  void _init(void) override;

  sh2_Hal_t _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
  sh2_SensorValue_t buffer;
};

#endif // end of _I2C_BNO08X_H
