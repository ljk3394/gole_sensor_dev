// g++ -Wall -pthread -o main main.cpp ./src/* -I ./include -lpigpiod_if2 -lrt
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <pigpiod_if2.h>
// #include <mutex>
#include "i2c_device.h"
#include "imu.h"
#include "encoder.h"

constexpr uint8_t BUS_NUM = 1; // #define BUS_NUM 1
// std::mutex mtx;


void test_imu(ISM330DHCX& imu, size_t N, float dt) {
  float ax, ay, az, gx, gy, gz;

  cout << "chipID = " << dec2hex(imu.chipID()) << endl;
  imu.print_accel_range();
  imu.print_gyro_range();
  imu.print_accel_datarate();
  imu.print_gyro_datarate();

  for(size_t i=0; i < N; i++) {
    imu.readAcceleration(ax, ay, az);
    imu.readGyroscope(gx, gy, gz);

    cout << "[Accel:" << imu.accelerationAvailable() << ",Gyro:" << imu.accelerationAvailable() << "," << dec2hex(imu.status()) << "]: ";
    cout << "[" << ax << ", " << ay << ", " << az << ",     " << gx << ", " << gy << ", " << gz << "]" << endl;
    // cout << "Status: " << imu.dec2hex(imu.status()) << endl;
    // imu.readAcceleration(ax, ay, az);
    // cout << "Accel available?: " << imu.accelerationAvailable() << ", [" << x << ", " << y << ", " << z << "]" << endl;
    // imu.readGyroscope(gx, gy, gz);
    // cout << "Gyro available?: " << imu.gyroscopeAvailable() << ", [" << x << ", " << y << ", " << z << "]" << endl;
  }

  usleep(dt * 1e6);
}

void test_encoder(vector<std::unique_ptr<AM4096>>& encs, const size_t N, const float dt) {
  auto t0 = chrono::system_clock::now();
  auto t1 = chrono::system_clock::now();
  chrono::duration<double> elapsed;

  cout << "[test_encoder] Start!" << endl;
  for(size_t i=0; i < N; i++) {
    t1 = chrono::system_clock::now(); elapsed = t1 - t0; t0 = t1;

    for(std::unique_ptr<AM4096>& enc: encs) {
      enc->update();
    }
    usleep(dt * 1e6);
  }
  cout << "[test_encoder] Done!" << endl;
}

void test_single_encoder(int i2c_dev, unsigned bus_num, unsigned dev_id, int N, float dt) {
  int i2c_handle_enc = i2c_open(i2c_dev, bus_num, dev_id, 0);
  cout << "[test_single_encoder] Start!" << endl;

  for (int i=0; i < N ; i++) {
    i2c_read_word_data(i2c_dev, i2c_handle_enc, 32);
    usleep(dt * 1e6);
  }
  i2c_close(i2c_dev, i2c_handle_enc);
  cout << "[test_single_encoder] Done!" << endl;
}

void test_chain(vector<std::unique_ptr<I2CDevice>>& devs, const size_t N, const float dt, const unsigned bus_num=0) {
  auto t00 = chrono::system_clock::now();
//  auto t0 = chrono::system_clock::now();
//  auto t1 = chrono::system_clock::now();
//  chrono::duration<double> elapsed;

  // mtx.lock();
  cout << "[test_chain] Start!" << endl;
  // mtx.unlock();

  for(size_t i=0; i < N; i++) {
    for(std::unique_ptr<I2CDevice>& d: devs) {
      d->update();

      // if(typeid(AM4096) == typeid(*d)) {
      //   AM4096* enc = dynamic_cast<AM4096*>(d.get());
      //   cout << "[AM4096:" << dec2hex(enc->id) << "] pos[" << enc->valid << "] :" << enc->pos << endl;
      // } else if (typeid(ISM330DHCX) == typeid(*d)) {
      //   ISM330DHCX* imu = dynamic_cast<ISM330DHCX*>(d.get());
      //   cout << "[IMU:" << dec2hex(imu->id) << "] acc:(" << imu->acc[0] << ", " << imu->acc[1] << ", " << imu->acc[2] << ")";
      //   cout << ", gyro: (" << imu->gyro[0] << ", " << imu->gyro[1] << ", " << imu->gyro[2] << ")";
      //   cout << ", temp: " << imu->temperature << "\t";
      // }
    }

    // t1 = chrono::system_clock::now(); elapsed = t1 - t0; t0 = t1;
    // cout << endl << "\t" << 1/(elapsed.count()) << " Hz" << endl;

    if(dt > 0) usleep(dt * 1e6);
  }

  auto t11 = chrono::system_clock::now();
  chrono::duration<double> total_elapsed = t11 - t00;

  // mtx.lock();
  cout << "[test_entest_chaincoder:" << bus_num << "] Done! " << 1000.0 * total_elapsed.count() / N << " ms, " << 1.0 * N / total_elapsed.count() << " Hz for " << N << " samples." << endl;
  // mtx.unlock();

}

int main() {
  int i2c_dev = pigpio_start(nullptr, nullptr);
  if(i2c_dev < 0) {
    cerr << "cannot connect to pigpiod!" << endl;
    exit(EXIT_FAILURE);
  }

  // // Test IMU
  // ISM330DHCX imu(i2c_dev, BUS_NUM, IMU_ADDR); devs.push_back(imu);
  // if(imu.i2c_handle >= 0) {
  //   test_imu(imu, 1000, 100e-3);
  // } else {
  //   cerr << "Cannot find the i2c device!" << endl;
  // }

  // // Test AM4096
  // vector<std::unique_ptr<AM4096>> encs;
  // encs.push_back(std::make_unique<AM4096>(i2c_dev, BUS_NUM, 0x11));
  // encs.push_back(std::make_unique<AM4096>(i2c_dev, BUS_NUM, 0x12));
  // encs.push_back(std::make_unique<AM4096>(i2c_dev, BUS_NUM, 0x13));
  // encs.push_back(std::make_unique<AM4096>(i2c_dev, BUS_NUM, 0x14));
  //
  // if (encs[0]->i2c_handle >= 0) {
  //   test_encoder(encs, 5, 100e-3);
  // } else {
  //   cerr << "Cannot find the i2c device!" << endl;
  // }

  // Test AM4096
  // vector<std::unique_ptr<I2CDevice>> devs;
  // devs.push_back(std::make_unique<ISM330DHCX>(i2c_dev, 1, IMU_ADDR));
  // devs.push_back(std::make_unique<AM4096>(i2c_dev, 1, 0x11));
  // devs.push_back(std::make_unique<AM4096>(i2c_dev, 1, 0x12));
  // devs.push_back(std::make_unique<AM4096>(i2c_dev, BUS_NUM, 0x13));
  // devs.push_back(std::make_unique<AM4096>(i2c_dev, BUS_NUM, 0x14));

  unsigned N = 1000;
  float dt = -0.0;// 100e-3;

  // if (devs[0]->i2c_handle >= 0) {
  //   test_chain(devs, N, dt);
  // } else {
  //   cerr << "Cannot find the i2c device!" << endl;
  // }

  // Test multi-threading
  vector<std::unique_ptr<I2CDevice>> devs1;
  // devs1.push_back(std::make_unique<ISM330DHCX>(i2c_dev, 1, IMU_ADDR));
  // devs1.push_back(std::make_unique<AM4096>(i2c_dev, 1, 0x11));
  // devs1.push_back(std::make_unique<AM4096>(i2c_dev, 1, 0x12));
  // devs1.push_back(std::make_unique<AM4096>(i2c_dev, 1, 0x13));
  // devs1.push_back(std::make_unique<AM4096>(i2c_dev, 1, 0x14));

  vector<std::unique_ptr<I2CDevice>> devs3;
  // devs3.push_back(std::make_unique<ISM330DHCX>(i2c_dev, 3, IMU_ADDR));
  devs3.push_back(std::make_unique<AM4096>(i2c_dev, 3, 0x11));
  devs3.push_back(std::make_unique<AM4096>(i2c_dev, 3, 0x12));
  // devs3.push_back(std::make_unique<AM4096>(i2c_dev, 3, 0x13));
  devs3.push_back(std::make_unique<AM4096>(i2c_dev, 3, 0x14));

  vector<std::unique_ptr<I2CDevice>> devs4;
  devs4.push_back(std::make_unique<ISM330DHCX>(i2c_dev, 4, IMU_ADDR));
  // devs4.push_back(std::make_unique<AM4096>(i2c_dev, 4, 0x11));
  // devs4.push_back(std::make_unique<AM4096>(i2c_dev, 4, 0x12));
  devs4.push_back(std::make_unique<AM4096>(i2c_dev, 4, 0x13));
  // devs4.push_back(std::make_unique<AM4096>(i2c_dev, 4, 0x14));

  // std::thread test_chain_1([&]() {test_chain(devs1, N, dt, 1);});
  std::thread test_chain_3([&]() {test_chain(devs3, N, dt, 3);});
  std::thread test_chain_4([&]() {test_chain(devs4, N, dt, 4);});

  // test_chain_1.join();
  test_chain_3.join();
  test_chain_4.join();

  // cleaning up before close the app.
  cout << "Cleaning up...";
  pigpio_stop(i2c_dev);
  cout << " Done." << endl;
}
