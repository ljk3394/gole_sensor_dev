// g++ -Wall -pthread -o imu imu.cpp -lpigpiod_if2 -lrt
#include <iostream>
#include <unistd.h>
#include <cstdlib> // exit
#include <cmath> //NAN

#include "imu.h"
#include "lsm6ds_consts.h"
#include <pigpiod_if2.h>

using namespace std;

/*!
 *    @brief  Instantiates a new ISM330DHCX class
 */
ISM330DHCX::ISM330DHCX(int i2c_dev, int8_t bus_num, int16_t dev_id)
    : I2CDevice(i2c_dev, bus_num, dev_id) {
      configInt1(false, false, true); // accelerometer DRDY on INT1
      configInt2(false, true, false); // gyro DRDY on INT2
      _init();
}

/*!
 *    @brief  Cleans up the ISM330DHCX
 */
ISM330DHCX::~ISM330DHCX(void) {}

/*!  @brief  Unique subclass initializer post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
void ISM330DHCX::_init(void) {
  // Enable accelerometer with 104 Hz data rate, 4G
  setAccelDataRate(LSM6DS_RATE_416_HZ);
  setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  // Enable gyro with 104 Hz data rate, 2000 dps
  setGyroDataRate(LSM6DS_RATE_416_HZ);
  setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  sleep(0.5);
}

// read ChipID value
uint8_t ISM330DHCX::chipID(void) {
  return _read(LSM6DS_WHOAMI, 1);
}

uint8_t ISM330DHCX::status(void) {
  return _read(LSM6DS_STATUS_REG, 1);
}

void ISM330DHCX::reset(void) {
  _write(LSM6DS_CTRL3_C, 1, 1, 1, 0);
  while(_read(LSM6DS_CTRL3_C, 1, 1, 0)) {
    sleep(1);
  }
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer data rate.
    @returns The the accelerometer data rate.
*/
lsm6ds_data_rate_t ISM330DHCX::getAccelDataRate(void) {
  return (lsm6ds_data_rate_t)_read(LSM6DS_CTRL1_XL, 1, 4, 4);
}

/**************************************************************************/
/*!
    @brief Sets the accelerometer data rate.
    @param  data_rate
            The the accelerometer data rate. Must be a `lsm6ds_data_rate_t`.
*/
void ISM330DHCX::setAccelDataRate(lsm6ds_data_rate_t data_rate) {
  _write(LSM6DS_CTRL1_XL, 1, data_rate, 4, 4);
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer measurement range.
    @returns The the accelerometer measurement range.
*/
lsm6ds_accel_range_t ISM330DHCX::getAccelRange(void) {
  accelRangeBuffered = (lsm6ds_accel_range_t)_read(LSM6DS_CTRL1_XL, 1, 2, 2);
  return accelRangeBuffered;
}
/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range.
    @param new_range The `lsm6ds_accel_range_t` range to set.
*/
void ISM330DHCX::setAccelRange(lsm6ds_accel_range_t new_range) {
  _write(LSM6DS_CTRL1_XL, 1, new_range, 2, 2);
  accelRangeBuffered = new_range;
}

/**************************************************************************/
/*!
    @brief Gets the gyro data rate.
    @returns The the gyro data rate.
*/
lsm6ds_data_rate_t ISM330DHCX::getGyroDataRate(void) {
  return (lsm6ds_data_rate_t)_read(LSM6DS_CTRL2_G, 1, 4, 4);
}

/**************************************************************************/
/*!
    @brief Sets the gyro data rate.
    @param  data_rate
            The the gyro data rate. Must be a `lsm6ds_data_rate_t`.
*/
void ISM330DHCX::setGyroDataRate(lsm6ds_data_rate_t data_rate) {
  _write(LSM6DS_CTRL2_G, 1, data_rate, 4, 4);
}

/**************************************************************************/
/*!
    @brief Gets the gyro range.
    @returns The the gyro range.
*/
lsm6ds_gyro_range_t ISM330DHCX::getGyroRange(void) {
  gyroRangeBuffered = (lsm6ds_gyro_range_t)_read(LSM6DS_CTRL2_G, 1, 4, 0);
  return gyroRangeBuffered;
}

/**************************************************************************/
/*!
    @brief Sets the gyro range.
    @param new_range The `lsm6ds_gyro_range_t` to set.
*/
void ISM330DHCX::setGyroRange(lsm6ds_gyro_range_t new_range) {
  _write(LSM6DS_CTRL2_G, 1, new_range, 4, 0);
  gyroRangeBuffered = new_range;
}

/**************************************************************************/
/*!
    @brief Enables the high pass filter and/or slope filter
    @param filter_enabled Whether to enable the slope filter (see datasheet)
    @param filter The lsm6ds_hp_filter_t that sets the data rate divisor
*/
/**************************************************************************/
void ISM330DHCX::highPassFilter(bool filter_enabled, lsm6ds_hp_filter_t filter) {
  _write(LSM6DS_CTRL8_XL, 1, filter_enabled, 1, 2); // HPF_en
  _write(LSM6DS_CTRL8_XL, 1, filter, 2, 5);         // HPF_filter
}


/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void ISM330DHCX::update(void) {
  // get raw readings
  uint8_t buffer[14];
  _read(LSM6DS_OUT_TEMP_L, sizeof(buffer), buffer); // 14 values

  // rawTemp = buffer[1] << 8 | buffer[0];
  // temperature = (rawTemp / temperature_sensitivity) + 25.0;

  // rawGyro[0] = buffer[3] << 8 | buffer[2];
  // rawGyro[1] = buffer[5] << 8 | buffer[4];
  // rawGyro[2] = buffer[7] << 8 | buffer[6];

  // rawAcc[0] = buffer[9] << 8 | buffer[8];
  // rawAcc[1] = buffer[11] << 8 | buffer[10];
  // rawAcc[2] = buffer[13] << 8 | buffer[12];

  rawTemp = buffer[12] << 8 | buffer[13];
  temperature = (rawTemp / temperature_sensitivity) + 25.0;

  rawGyro[0] = buffer[10] << 8 | buffer[11];
  rawGyro[1] = buffer[8] << 8 | buffer[9];
  rawGyro[2] = buffer[6] << 8 | buffer[7];

  rawAcc[0] = buffer[4] << 8 | buffer[5];
  rawAcc[1] = buffer[2] << 8 | buffer[3];
  rawAcc[2] = buffer[0] << 8 | buffer[1];

  float gyro_scale = 1; // range is in milli-dps per bit!
  switch (gyroRangeBuffered) {
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    gyro_scale = 140.0;
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    gyro_scale = 70.0;
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    gyro_scale = 35.0;
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    gyro_scale = 17.50;
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    gyro_scale = 8.75;
    break;
  case LSM6DS_GYRO_RANGE_125_DPS:
    gyro_scale = 4.375;
    break;
  }

  gyro[0] = rawGyro[0] * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
  gyro[1] = rawGyro[1] * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
  gyro[2] = rawGyro[2] * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;

  float accel_scale = 1; // range is in milli-g per bit!
  switch (accelRangeBuffered) {
  case LSM6DS_ACCEL_RANGE_16_G:
    accel_scale = 0.488;
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    accel_scale = 0.244;
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    accel_scale = 0.122;
    break;
  case LSM6DS_ACCEL_RANGE_2_G:
    accel_scale = 0.061;
    break;
  }

  acc[0] = rawAcc[0] * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
  acc[1] = rawAcc[1] * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
  acc[2] = rawAcc[2] * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
}

/**************************************************************************/
/*!
    @brief Sets the INT1 and INT2 pin activation mode
    @param active_low true to set the pins  as active high, false to set the
   mode to active low
    @param open_drain true to set the pin mode as open-drain, false to set the
   mode to push-pull
*/
void ISM330DHCX::configIntOutputs(bool active_low, bool open_drain) {
  _write(LSM6DS_CTRL3_C, 1, (active_low << 1) | open_drain, 2, 4); // ppod_bits
}

/**************************************************************************/
/*!
    @brief Enables and disables the data ready interrupt on INT 1.
    @param drdy_temp true to output the data ready temperature interrupt
    @param drdy_g true to output the data ready gyro interrupt
    @param drdy_xl true to output the data ready accelerometer interrupt
    @param step_detect true to output the step detection interrupt (default off)
    @param wakeup true to output the wake up interrupt (default off)
*/
void ISM330DHCX::configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl, bool step_detect, bool wakeup) {
  _write(LSM6DS_INT1_CTRL, 1, (step_detect << 7) | (drdy_temp << 2) | (drdy_g << 1) | drdy_xl);
  _write(LSM6DS_MD1_CFG, 1, wakeup, 1, 5);
}

/**************************************************************************/
/*!
    @brief Enables and disables the data ready interrupt on INT 2.
    @param drdy_temp true to output the data ready temperature interrupt
    @param drdy_g true to output the data ready gyro interrupt
    @param drdy_xl true to output the data ready accelerometer interrupt
*/
void ISM330DHCX::configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl) {
  _write(LSM6DS_INT2_CTRL, (drdy_temp << 2) | (drdy_g << 1) | drdy_xl, 3, 0);
}


/**************************************************************************/
/*!
    @brief Gets the accelerometer data rate.
    @returns The data rate in float
*/
float ISM330DHCX::accelerationSampleRate(void) {
  return _data_rate_arr[this->getAccelDataRate()];
}

/**************************************************************************/
/*!
    @brief Check for available data from accelerometer
    @returns 1 if available, 0 if not
*/
int ISM330DHCX::accelerationAvailable(void) {
  return (this->status() & 0x01) ? 1 : 0;
}

/**************************************************************************/
/*!
    @brief Read accelerometer data
    @param x reference to x axis
    @param y reference to y axis
    @param z reference to z axis
    @returns 1 if success, 0 if not
*/
int ISM330DHCX::readAcceleration(float &x, float &y, float &z) {
  int16_t data[3];

  if(_read(LSM6DS_OUTX_L_A, sizeof(data), (uint8_t *)data) < 0) { // 6 values
    x = y = z = NAN;
    return 0;
  }

  // scale to range of -4 – 4
  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;

  return 1;
}

/**************************************************************************/
/*!
    @brief Get the gyroscope data rate.
    @returns The data rate in float
*/
float ISM330DHCX::gyroscopeSampleRate(void) {
  return _data_rate_arr[this->getGyroDataRate()];
}

/**************************************************************************/
/*!
    @brief Check for available data from gyroscope
    @returns 1 if available, 0 if not
*/
int ISM330DHCX::gyroscopeAvailable(void) {
  return (this->status() & 0x02) ? 1 : 0;
}

/**************************************************************************/
/*!
    @brief Read gyroscope data
    @param x reference to x axis
    @param y reference to y axis
    @param z reference to z axis
    @returns 1 if success, 0 if not
*/
int ISM330DHCX::readGyroscope(float &x, float &y, float &z) {
  int16_t data[3];

  if(_read(LSM6DS_OUTX_L_G, sizeof(data), (uint8_t *)data) < 0) { // 6 values
    x = y = z = NAN;
    return 0;
  }

  // scale to range of -2000 – 2000
  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}






void ISM330DHCX::print_accel_range() {
  string s;
  cout << "Accelerometer range set to: ";
  switch (getAccelRange()) {
    case LSM6DS_ACCEL_RANGE_2_G : s = "+-2G"; break;
    case LSM6DS_ACCEL_RANGE_4_G : s = "+-4G"; break;
    case LSM6DS_ACCEL_RANGE_8_G : s = "+-8G"; break;
    case LSM6DS_ACCEL_RANGE_16_G: s = "+-16G"; break;
  }
  cout << s << endl;
}

void ISM330DHCX::print_gyro_range() {
  string s;
  cout << "Gyro range set to: ";
  switch (getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS     : s = "125 degrees/s"; break;
    case LSM6DS_GYRO_RANGE_250_DPS     : s = "250 degrees/s"; break;
    case LSM6DS_GYRO_RANGE_500_DPS     : s = "500 degrees/s"; break;
    case LSM6DS_GYRO_RANGE_1000_DPS    : s = "1000 degrees/s"; break;
    case LSM6DS_GYRO_RANGE_2000_DPS    : s = "2000 degrees/s"; break;
    case ISM330DHCX_GYRO_RANGE_4000_DPS: s = "4000 degrees/s"; break;
  }

  cout << s << endl;
}

void ISM330DHCX::print_accel_datarate() {
  string s;
  cout << "Accelerometer data rate set to: ";

  switch (getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN: s = "0 Hz"; break;
    case LSM6DS_RATE_12_5_HZ : s = "12.5 Hz"; break;
    case LSM6DS_RATE_26_HZ   : s = "26 Hz"; break;
    case LSM6DS_RATE_52_HZ   : s = "52 Hz"; break;
    case LSM6DS_RATE_104_HZ  : s = "104 Hz"; break;
    case LSM6DS_RATE_208_HZ  : s = "208 Hz"; break;
    case LSM6DS_RATE_416_HZ  : s = "416 Hz"; break;
    case LSM6DS_RATE_833_HZ  : s = "833 Hz"; break;
    case LSM6DS_RATE_1_66K_HZ: s = "1.66 KHz"; break;
    case LSM6DS_RATE_3_33K_HZ: s = "3.33 KHz"; break;
    case LSM6DS_RATE_6_66K_HZ: s = "6.66 KHz"; break;
  }

  cout << s << endl;
}

void ISM330DHCX::print_gyro_datarate() {
  string s;
  cout << "Gyro data rate set to: ";
  switch (getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN: s = "0 Hz"; break;
    case LSM6DS_RATE_12_5_HZ : s = "12.5 Hz"; break;
    case LSM6DS_RATE_26_HZ   : s = "26 Hz"; break;
    case LSM6DS_RATE_52_HZ   : s = "52 Hz"; break;
    case LSM6DS_RATE_104_HZ  : s = "104 Hz"; break;
    case LSM6DS_RATE_208_HZ  : s = "208 Hz"; break;
    case LSM6DS_RATE_416_HZ  : s = "416 Hz"; break;
    case LSM6DS_RATE_833_HZ  : s = "833 Hz"; break;
    case LSM6DS_RATE_1_66K_HZ: s = "1.66 KHz"; break;
    case LSM6DS_RATE_3_33K_HZ: s = "3.33 KHz"; break;
    case LSM6DS_RATE_6_66K_HZ: s = "6.66 KHz"; break;
  }

  cout << s << endl;
}

int ISM330DHCX::ping(bool verbose) {
  int ret = i2c_read_byte_data(i2c_dev, i2c_handle, LSM6DS_WHOAMI);

  if(ret >= 0) {
    if(verbose) cout << "[ISM330DHCX(" << dec2hex(id) << ")::ping] OK(" << ret << ")" << endl;
    return 1;
  } else {
    if(verbose) cout << "[ISM330DHCX(" << dec2hex(id) << ")::ping] Error(" << ret << ") occurred - " << getI2CErrorMessage(ret) << endl;
    return 0;
  }
}