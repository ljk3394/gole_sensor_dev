#include <iostream>
#include <unistd.h>
#include <cstdlib> // exit
#include <cmath> //NAN
#include <ctime>
#include <cstring>
#include <chrono>

#include "bno08x.h"
#include <pigpiod_if2.h>

// Function to interface with SH2 library
static int8_t _reset_pin;
static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static void i2chal_close(sh2_Hal_t *self);
static int i2chal_open(sh2_Hal_t *self);

static void hal_hardwareReset(void);
static uint32_t hal_getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);

// communication wrapper
static BNO08X* _bno08x = nullptr;
int32_t BNO08X::read(unsigned count, uint8_t* buf) { return _readDev(count, buf); }
int32_t BNO08X::write(unsigned count, uint32_t val, uint8_t* buf) { return _writeDev(count, val, buf); }

BNO08X::BNO08X(int i2c_dev, int bus_num, int16_t dev_id, long period, int8_t reset_pin): BNO08X(i2c_dev, bus_num, dev_id, period, period, period, period, period, reset_pin) {}
BNO08X::BNO08X(int i2c_dev, int bus_num, int16_t dev_id, long periodOri, long periodAcc, long periodGyro, long periodGravity, long periodMag, int8_t reset_pin): I2CDevice(i2c_dev, bus_num, dev_id), periodOri(periodOri), periodAcc(periodAcc), periodGyro(periodGyro), periodGravity(periodGravity), periodMag(periodMag) { _reset_pin = reset_pin; _bno08x = this; _init();}
BNO08X::~BNO08X(void) {} // if (temp_sensor) delete temp_sensor;

void BNO08X::update(void) {
  // static uint64_t t0_rv = static_cast<uint64_t> (time(nullptr) * 1000);
  // static uint64_t t0_la = static_cast<uint64_t> (time(nullptr) * 1000);
  // static uint64_t t0_g  = static_cast<uint64_t> (time(nullptr) * 1000);
  // static uint64_t t0_gy = static_cast<uint64_t> (time(nullptr) * 1000);
  // static uint64_t t0_mg = static_cast<uint64_t> (time(nullptr) * 1000);
  // static size_t idx_rv = 0;
  // static size_t idx_la = 0;
  // static size_t idx_g = 0;
  // static size_t idx_gy = 0;
  // static size_t idx_mg = 0;
  // uint64_t t1 = static_cast<uint64_t> (time(nullptr) * 1000);
  
  if(wasReset()) {
    cout << "[BNO08X] Sensor was reset" << endl;
    if (! enableReports()) { cout << "[BNO08X] Could not enable reports...!" << endl; }
  }

  buffer.timestamp = 0;
  sh2_service();
  // if (buffer.timestamp == 0 && buffer.sensorId != SH2_GYRO_INTEGRATED_RV) return;  
  if(buffer.sensorId == 0) return; // no new events
  if(buffer.status > 3) return; // wrong trash values

  quality = buffer.status;
  switch (buffer.sensorId) {
    case SH2_ROTATION_VECTOR:
        accuracy = buffer.un.rotationVector.accuracy;
        qt[0] = buffer.un.rotationVector.real;
        qt[1] = buffer.un.rotationVector.i;
        qt[2] = buffer.un.rotationVector.j;
        qt[3] = buffer.un.rotationVector.k;
        
        // idx_rv = (idx_rv + 1) % 100;
        // if(idx_rv == 1)  {
        //   cout << 0.01 * (t1 - t0_rv) << "ms, Rotation Vector received: " << "(" << qt[0] << "," << qt[1] << "," << qt[2] << "," << qt[3] << ")" << endl;
        //   t0_rv = t1;
        // }
        break;

    case SH2_LINEAR_ACCELERATION: // faster (more noise?)
        acc[0] = buffer.un.accelerometer.x;
        acc[1] = buffer.un.accelerometer.y;
        acc[2] = buffer.un.accelerometer.z;
        
        // idx_la = (idx_la + 1) % 100;
        // if(idx_la == 1){
        //   cout << 0.01 * (t1 - t0_la) << "ms, Linear Acceleration received: " << "(" << acc[0] << "," << acc[1] << "," << acc[2] << ")" << endl;
        //   t0_la = t1; 
        // }
        break;

    case SH2_GRAVITY:
        g[0] = buffer.un.gravity.x;
        g[1] = buffer.un.gravity.y;
        g[2] = buffer.un.gravity.x;
        
        // idx_g = (idx_g + 1) % 100;
        // if(idx_g == 1) {
        //   cout << 0.01 * (t1 - t0_g) << "ms, Gravity Vector received: " << "(" << g[0] << "," << g[1] << "," << g[2] << ")" << endl;
        //   t0_g = t1;
        // }
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        gyro[0] = buffer.un.gyroscope.x;
        gyro[1] = buffer.un.gyroscope.y;
        gyro[2] = buffer.un.gyroscope.z;
        
        // idx_gy = (idx_gy + 1) % 100;
        // if(idx_gy == 1) {
        //   cout << 0.01 * (t1 - t0_gy) << "ms, gyro received: " << "(" << gyro[0] << "," << gyro[1] << "," << gyro[2] << ")" << endl;
        //   t0_gy = t1;
        // }
        break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
        mag[0] = buffer.un.magneticField.x;
        mag[1] = buffer.un.magneticField.y;
        mag[2] = buffer.un.magneticField.z;

        // idx_mg = (idx_mg + 1) % 100;
        // if(idx_mg == 1) {
        //   cout << 0.01 * (t1 - t0_mg) << "ms, gyro received: " << "(" << mag[0] << "," << mag[1] << "," << mag[2] << ")" << endl;
        //   t0_mg = t1;
        // }

        break;
  }
}

void BNO08X::_init(void) {
  int status;
  
  _sensor_value = &buffer;

  _HAL.open = i2chal_open;
  _HAL.close = i2chal_close;
  _HAL.read = i2chal_read;
  _HAL.write = i2chal_write;
  _HAL.getTimeUs = hal_getTimeUs;

  reset();

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&_HAL, hal_callback, NULL);
  if (status != SH2_OK) { throw runtime_error("Cannot open SH2 Interface"); }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) { throw runtime_error("Cannot connect to SH2 Device"); }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL); 

  // Setting Desired Reports (make it run!)
  //   report types: there are many more, but only listed those useful
  //     reference: https://github.com/adafruit/Adafruit_BNO08x/blob/master/src/sh2.h
  //                https://github.com/adafruit/Adafruit_BNO08x/blob/master/src/sh2_SensorValue.h
  //                https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf
  // SH2_ACCELEROMETER [m/s^2]; SH2_LINEAR_ACCELERATION + SH2_GRAVITY, 
  // SH2_GYROSCOPE_CALIBRATED [rad/s]
  // SH2_MAGNETIC_FIELD_CALIBRATED [uT]
  // SH2_ROTATION_VECTOR [quaternion]
  
  if (! enableReports()) { cout << "[BNO08X] Could not enable reports...!" << endl; }
}

int BNO08X::ping(bool verbose) { 
  memset(&prodIds, 0, sizeof(prodIds));
  int status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) { 
    return 0; 
  } else {
    return 1;
  }
}
void BNO08X::reset(void) { hal_hardwareReset(); }
bool BNO08X::wasReset(void) {bool x = _reset_occurred; _reset_occurred = false; return x;}

bool BNO08X::enableReports(void) {
  bool res = true;

  if (periodOri <= 0) {
    // cout << "[BNO08X] ROTATION_VECTOR disabled." << endl;
  } else if (!enableReport(SH2_GYRO_INTEGRATED_RV, periodOri)) { // originaly, SH2_ROTATION_VECTOR for the best performance in terms of accuracy
    cout << "[BNO08X] Could not enable ROTATION_VECTOR." << endl;
    res = false;
  }

  if (periodAcc <= 0) {
    // cout << "[BNO08X] LINEAR_ACCELERATION disabled." << endl;
  } else if (!enableReport(SH2_LINEAR_ACCELERATION, periodAcc)) {
    cout << "[BNO08X] Could not enable LINEAR_ACCELERATION." << endl;
    res = false;
  }

  if (periodGravity <= 0) {
    // cout << "[BNO08X] GRAVITY disabled." << endl;
  } else if (!enableReport(SH2_GRAVITY, periodGravity)) {
    cout << "[BNO08X] Could not enable GRAVITY." << endl;
    res = false;
  }
  if (periodGyro <= 0) {
    // cout << "[BNO08X] GYROSCOPE_CALIBRATED disabled." << endl;
  } else if (!enableReport(SH2_GYROSCOPE_CALIBRATED, periodGyro)) {
    cout << "[BNO08X] Could not enable GYROSCOPE_CALIBRATED." << endl;
    res = false;
  }

  if (periodMag <= 0) {
    // cout << "[BNO08X] MAGNETIC_FIELD_CALIBRATED disabled." << endl;
  } else if (!enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, periodMag)) {
    cout << "[BNO08X] Could not enable MAGNETIC_FIELD_CALIBRATED." << endl;
    res = false;
  }

  return res;
}
bool BNO08X::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);
  if (status != SH2_OK) { return false; }

  return true;
}

/**************************************** I2C interface ***********************************************************/
static int i2chal_open(sh2_Hal_t *self) {
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    if (_bno08x->write(5, 0, softreset_pkt) > 0) {
        success = true;
        break;
    }
    usleep(30000);
  }
  if (!success)
    return -1;
  usleep(300000);
  return 0;
}

static void i2chal_close(sh2_Hal_t *self) {}

static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
  uint8_t header[4];
  if (_bno08x->read(4, header) <= 0) {
    return 0;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  size_t i2c_buffer_max = BUFFER_SIZE;

  if (packet_size > len) {
    // packet wouldn't fit in our buffer
    return 0;
  }
  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

    if(_bno08x->read(read_size, i2c_buffer) <= 0) {
      return 0;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of the i2c buffer to skip the header included with 
      // every new i2c read and don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    
    pBuffer += cargo_read_amount; // advance our pointer by the amount of cargo read
    cargo_remaining -= cargo_read_amount; // mark the cargo as received
  }

  return packet_size;
}

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  size_t i2c_buffer_max = BUFFER_SIZE;
  uint16_t write_size = min(i2c_buffer_max, len);

  int sentByte = _bno08x->write(write_size, 0, pBuffer);
  if(sentByte <= 0) {
    return 0;
  }

  return write_size;
}

/**************************************** HAL interface ************************************************************/
static void hal_hardwareReset(void) {
//   if (_reset_pin != -1) {
//     pinMode(_reset_pin, OUTPUT);
//     digitalWrite(_reset_pin, HIGH); delay(10);
//     digitalWrite(_reset_pin, LOW);  delay(10);
//     digitalWrite(_reset_pin, HIGH); delay(10);
//   }
    cout << "RESET not implemented" << endl;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  uint32_t t = time(nullptr) * 1000;
  return t;
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    cout << "BNO08x - Error decoding sensor event" << endl;
    _sensor_value->timestamp = 0;
    return;
  }
}
/****************************************************************************************************/