#ifndef IMU_ISM330DHCX_H
#define IMU_ISM330DHCX_H

#include "i2c_device.h"
#include "lsm6ds_consts.h"

#define IMU_ADDR LSM6DS_I2CADDR_DEFAULT

using namespace std;

class ISM330DHCX : public I2CDevice {
private:
  lsm6ds_accel_range_t accelRangeBuffered = LSM6DS_ACCEL_RANGE_2_G; //! buffer for the accelerometer range
  lsm6ds_gyro_range_t gyroRangeBuffered = LSM6DS_GYRO_RANGE_250_DPS; //! buffer for the gyroscope range
  float temperature_sensitivity = 256.0; ///< Temp sensor sensitivity in LSB/degC

  void _init (void) override;

public:
  int16_t id = LSM6DS_I2CADDR_DEFAULT;
  // Sensor values
  int16_t rawAcc[3];    ///< Last reading's raw accelerometer X, Y, Z axis
  int16_t rawTemp;      ///< Last reading's raw temperature reading
  int16_t rawGyro[3];   ///< Last reading's raw gyro X, Y, Z axis
  float temperature;          ///< Last reading's temperature (C)
  float acc[3];         ///< Last reading's accelerometer X, Y, Z axis m/s^2
  float gyro[3];        ///< Last reading's gyro X, Y, Z axis in rad/s

  ISM330DHCX(int i2c_dev, int8_t i2c_handle, int16_t dev_id=IMU_ADDR);
  ~ISM330DHCX();

  void reset (void) override;
  void update (void) override;
  int ping(bool verbose=false) override;

  uint8_t chipID(void);
  uint8_t status(void);

  // Sensor configuration getter and setters
  lsm6ds_accel_range_t getAccelRange(void);
  lsm6ds_data_rate_t getAccelDataRate(void);
  lsm6ds_gyro_range_t getGyroRange(void);
  lsm6ds_data_rate_t getGyroDataRate(void);
  void setAccelRange(lsm6ds_accel_range_t new_range);
  void setAccelDataRate(lsm6ds_data_rate_t data_rate);
  void setGyroRange(lsm6ds_gyro_range_t new_range);
  void setGyroDataRate(lsm6ds_data_rate_t data_rate);

  // Configurations
  void configIntOutputs(bool active_low, bool open_drain);
  void configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl, bool step_detect = false, bool wakeup = false);
  void configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl);
  void highPassFilter(bool enabled, lsm6ds_hp_filter_t filter);

  // Arduino compatible API
  int readAcceleration(float &x, float &y, float &z);
  int readGyroscope(float &x, float &y, float &z);
  float accelerationSampleRate(void);
  float gyroscopeSampleRate(void);
  int accelerationAvailable(void);
  int gyroscopeAvailable(void);

  // utilities
  void print_accel_range();
  void print_gyro_range();
  void print_accel_datarate();
  void print_gyro_datarate();
};

#endif //IMU_ISM330DHCX_H
