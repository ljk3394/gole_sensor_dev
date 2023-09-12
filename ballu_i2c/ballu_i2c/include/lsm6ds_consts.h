#ifndef LSM6DS_CONSTS_H
#define LSM6DS_CONSTS_H

/** The macro values from Adafruit_Sensor.h **/
#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)
#define SENSORS_DPS_TO_RADS (0.017453293F) /**< Degrees/s to rad/s multiplier */
#define SENSORS_RADS_TO_DPS (57.29577793F) /**< Rad/s to degrees/s  multiplier */

/** The macro values from Adafruit_LSM6DS.h **/
#define LSM6DS_I2CADDR_DEFAULT 0x6A ///< LSM6DS default i2c address

#define LSM6DS_FUNC_CFG_ACCESS 0x1 ///< Enable embedded functions register
#define LSM6DS_INT1_CTRL 0x0D      ///< Interrupt control for INT 1
#define LSM6DS_INT2_CTRL 0x0E      ///< Interrupt control for INT 2
#define LSM6DS_WHOAMI 0x0F         ///< Chip ID register
#define LSM6DS_CTRL1_XL 0x10       ///< Main accelerometer config register
#define LSM6DS_CTRL2_G 0x11        ///< Main gyro config register
#define LSM6DS_CTRL3_C 0x12        ///< Main configuration register
#define LSM6DS_CTRL8_XL 0x17       ///< High and low pass for accel
#define LSM6DS_CTRL10_C 0x19       ///< Main configuration register
#define LSM6DS_WAKEUP_SRC 0x1B     ///< Why we woke up
#define LSM6DS_STATUS_REG 0X1E     ///< Status register
#define LSM6DS_OUT_TEMP_L 0x20     ///< First data register (temperature low)
#define LSM6DS_OUTX_L_G 0x22       ///< First gyro data register
#define LSM6DS_OUTX_L_A 0x28       ///< First accel data register
#define LSM6DS_STEPCOUNTER 0x4B    ///< 16-bit step counter
#define LSM6DS_TAP_CFG 0x58        ///< Tap/pedometer configuration
#define LSM6DS_WAKEUP_THS 0x5B     ///< Single and double-tap function threshold register
#define LSM6DS_WAKEUP_DUR 0x5C     ///< Free-fall, wakeup, timestamp and sleep mode duration
#define LSM6DS_MD1_CFG 0x5E        ///< Functions routing on INT1 register

/** The accelerometer data rate */
typedef enum data_rate {
  LSM6DS_RATE_SHUTDOWN,
  LSM6DS_RATE_12_5_HZ,
  LSM6DS_RATE_26_HZ,
  LSM6DS_RATE_52_HZ,
  LSM6DS_RATE_104_HZ,
  LSM6DS_RATE_208_HZ,
  LSM6DS_RATE_416_HZ,
  LSM6DS_RATE_833_HZ,
  LSM6DS_RATE_1_66K_HZ,
  LSM6DS_RATE_3_33K_HZ,
  LSM6DS_RATE_6_66K_HZ,
} lsm6ds_data_rate_t;

/** The accelerometer data range */
typedef enum accel_range {
  LSM6DS_ACCEL_RANGE_2_G,
  LSM6DS_ACCEL_RANGE_16_G,
  LSM6DS_ACCEL_RANGE_4_G,
  LSM6DS_ACCEL_RANGE_8_G
} lsm6ds_accel_range_t;

/** The gyro data range */
typedef enum gyro_range {
  LSM6DS_GYRO_RANGE_125_DPS = 0b0010,
  LSM6DS_GYRO_RANGE_250_DPS = 0b0000,
  LSM6DS_GYRO_RANGE_500_DPS = 0b0100,
  LSM6DS_GYRO_RANGE_1000_DPS = 0b1000,
  LSM6DS_GYRO_RANGE_2000_DPS = 0b1100,
  ISM330DHCX_GYRO_RANGE_4000_DPS = 0b0001
} lsm6ds_gyro_range_t;

/** The high pass filter bandwidth */
typedef enum hpf_range {
  LSM6DS_HPF_ODR_DIV_50 = 0,
  LSM6DS_HPF_ODR_DIV_100 = 1,
  LSM6DS_HPF_ODR_DIV_9 = 2,
  LSM6DS_HPF_ODR_DIV_400 = 3,
} lsm6ds_hp_filter_t;

static const float _data_rate_arr[] = {
    [LSM6DS_RATE_SHUTDOWN] = 0.0f,    [LSM6DS_RATE_12_5_HZ] = 12.5f,
    [LSM6DS_RATE_26_HZ] = 26.0f,      [LSM6DS_RATE_52_HZ] = 52.0f,
    [LSM6DS_RATE_104_HZ] = 104.0f,    [LSM6DS_RATE_208_HZ] = 208.0f,
    [LSM6DS_RATE_416_HZ] = 416.0f,    [LSM6DS_RATE_833_HZ] = 833.0f,
    [LSM6DS_RATE_1_66K_HZ] = 1660.0f, [LSM6DS_RATE_3_33K_HZ] = 3330.0f,
    [LSM6DS_RATE_6_66K_HZ] = 6660.0f,
};

#endif // LSM6DS_CONSTS_H
