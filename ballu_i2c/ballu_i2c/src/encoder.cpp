// g++ -Wall -pthread -o encoder encoder.cpp -lpigpiod_if2 -lrt
#include <iostream>
#include <unistd.h>
#include <cstdlib> // exit
#include <cmath> //NAN

#include "encoder.h"
#include <pigpiod_if2.h>

AM4096::AM4096(int i2c_dev, int bus_num, int16_t dev_id, bool wrapToPi, bool inRadian)
    : wrapToPi(wrapToPi), inRadian(inRadian), I2CDevice(i2c_dev, bus_num, dev_id) {
    rangeMax = (inRadian)? M_PI: 180.0;
    _init();
}
AM4096::~AM4096(void) {}
void AM4096::_init() {}
void AM4096::reset() {}

void AM4096::update() {
  rawPos = _read(ADDR_POS_REL, 2); // rawPos as sensor reading
  // TODO: error handling, 44543 for raw reading -> with 0FFF, becomes 3583
  valid  = (0x8000 & rawPos) >> 15;
  rawPos = (0x0FFF & rawPos); // meaningful rawPos

  if(wrapToPi) {
      pos = wrap(rawPos / 4095.0 * (2 * rangeMax));
  } else {
      pos = rawPos / 4095.0 * (2 * rangeMax);
  }
}

uint16_t AM4096::getOffset(bool verbose) {
    uint16_t currentOffset = 0x0FFF & _read(ADDR_ZERO_POS, 2, 0, 12);
    if(verbose) std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::getOffset] current offset is " << currentOffset << std::endl;

    return currentOffset;
}
uint32_t AM4096::calibrateOffset(uint16_t offset, bool verbose) {
    uint16_t currentOffset = getOffset(verbose);

    if(verbose) {
        std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::calibrateOffset] trying to set offset to " << offset << std::endl;
    }

    uint32_t res = _write(ADDR_ZERO_POS, 2, 0x0FFF & offset, 0, 12);
    sleep(1);

    if(verbose) {
        currentOffset = 0x0FFF & _read(ADDR_ZERO_POS, 2, 0, 12);
        cout << "[AM4096::calibrateOffset] offset set to " << currentOffset << "." << endl;
    }
    return res;
}

uint32_t AM4096::calibrateOffset(bool verbose) { // 12 bits (0~11)
    // (0, 4095) = (0 deg, 360 deg)
    update();
    if(verbose) std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::calibrateOffset] current encoder reading is " << rawPos << std::endl;

    uint16_t currentOffset = getOffset(verbose);
    bool positiveAxis = _read(ADDR_ZERO_POS, 2, 12, 1) == 0;

    uint16_t newOffset = 0x0FFF & ( positiveAxis ? ((currentOffset + rawPos) % 4096) : ((currentOffset + (4096 - rawPos)) % 4096) );
    if(verbose) std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::calibrateOffset] trying to set offset to " << newOffset << std::endl;

    uint32_t res = _write(ADDR_ZERO_POS, 2, 0x0FFF & newOffset, 0, 12);
    sleep(1);

    if(verbose) {
        update();
        currentOffset = getOffset(false);
        std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::calibrateOffset] offset set to " << currentOffset << "." << std::endl;
        std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::calibrateOffset] current encoder reading is " << rawPos << std::endl;
    }
    return res;
}

uint8_t AM4096::getRotationSign(bool verbose) { // 1 bit (12)
    uint8_t currentAxis = _read(ADDR_ZERO_POS, 2, 12, 1);

    if(verbose) {
        std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::getRotationSign] current rotation sign is ";
        std::cout << (int)currentAxis << ", where 0: positive, 1: negative." << std::endl;
    }

    return currentAxis;
}

uint32_t AM4096::setRotationSign(bool positive, bool verbose) { // 1 bit (12)
    uint8_t rotAxis = positive? 0 : 1;
    uint8_t currentAxis = getRotationSign(verbose);

    if(verbose) std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::setRotationSign] trying to set rotation axis to " << (int)rotAxis << std::endl;
    uint32_t res = _write(ADDR_SIGN, 2, rotAxis, 12, 1);
    sleep(1);

    if(verbose) {
        currentAxis = _read(ADDR_ZERO_POS, 2, 12, 1);
        std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::setRotationSign] updated rotation axis is ";
        std::cout << (int)currentAxis << "." << std::endl;
    }
    return res;
}

void AM4096::updateMagnetStatus(bool verbose) {
    rawMagnet = _read(ADDR_MAGNET, 1, 5, 2);
    magnetDist[0] = (rawMagnet & 0b10) >> 1;
    magnetDist[1] = (rawMagnet & 0b01);

    if(verbose) {
        std::cout << "[AM4096(" << (int)bus_num << ":" << dec2hex(id) << ")::updateMagnetStatus] updated magnet status is (";
        std::cout << "far: " << magnetDist[0] << ", close: " << magnetDist[1] << "), where 0: normal, 1:abnormal." << std::endl;
    }
}

uint8_t AM4096::chipID(void) {
    return 0x7F & _read(ADDR_ID, 2, 0, 7);
}

int AM4096::ping(bool verbose) {
    int ret = i2c_read_word_data(i2c_dev, i2c_handle, ADDR_ID);
    if(ret >= 0) {
        if(verbose) cout << "[AM4096(" << dec2hex(id) << ")::ping] OK(" << ret << ")" << endl;
        return 1;
    } else {
        if(verbose) cout << "[AM4096(" << dec2hex(id) << ")::ping] Error(" << ret << ") occurred: " << getI2CErrorMessage(ret) << endl;
        return 0;
    }
}

void AM4096::updateId(unsigned new_id, float wait_sec, bool verbose) {
    if((new_id >> 7) > 0) {
        cout <<"[AM4096::updateId(" << dec2hex(id) << ")] WARNING: address is max 7-bit length. Given: " << new_id << endl;
        return;
    }

    cout << "[AM4096::updateId(" << dec2hex(id) << ")] writing " << new_id << " ..." << endl;
    int ret = _write(PERM_ADDR_ID, 1, new_id, 0, 7);
    
    if(ret < 0) {
        cout <<"[AM4096::updateId(" << dec2hex(id) << ")] Error (" << ret << ") occured: " << getI2CErrorMessage(ret) << endl;
    }

    usleep(wait_sec * 1e6);
}

float AM4096::wrap(float unwrapped) {
    // wraps [0...2pi] to [-pi...pi]
    unwrapped = fmod(unwrapped + rangeMax, 2 * rangeMax);
    return (unwrapped < 0) ? unwrapped + rangeMax : unwrapped - rangeMax;
}

float AM4096::unwrap(float wrapped) {
    wrapped = fmod(wrapped, 2 * rangeMax);
    return (wrapped < 0) ? wrapped + 2 * rangeMax : wrapped;
}