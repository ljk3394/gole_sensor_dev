//
// Created by Hosik Chae on 23. 2. 8.
//

#include <iostream>
#include <unistd.h>
#include <cstdlib> // exit
#include <cmath> //NAN
#include <chrono>
#include <memory>
#include <thread>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <vector>
#include <unordered_map>
#include <signal.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <mutex>

#include <yaml-cpp/yaml.h>

#include "argh.h"
#include "json.hpp"
#include "encoder.h"
#include "imu.h"
#include "bno08x.h"
#include "tof.h"
#include "plot_udp.h"
#include "shared_memory.tpp"

using namespace std;
using DevDict = unordered_map<string, string>;
using BusDict = unordered_map<uint32_t, DevDict>;
using Shmem3DVector = SharedMemoryBlock<double, 3>;
using Shmem4DVector = SharedMemoryBlock<double, 4>;
using ShmemJointVector = SharedMemoryBlock<double, 7>;

unordered_map<uint32_t, std::thread> threads;
unordered_map<unsigned, string> joint_names;
auto devs = unordered_map<uint32_t, std::shared_ptr<vector<std::shared_ptr<I2CDevice>>>>();

float dt = 0.0;
int i2c_dev;
bool verbose = false;
bool is_running = false;
bool plot_udp = false;
bool store_to_shm = false;
string shm_namespace;
unique_ptr<PlotUDPSocket> sock_udp;
shared_ptr<Shmem3DVector> shm_body_pos, shm_body_acc, shm_body_qt, shm_body_w;
shared_ptr<ShmemJointVector> shm_joint_q;
mutex shm_joint_q_lock;


void handle_yaml_arguments(int argc, char* argv[]);
void acquire(shared_ptr<vector<std::shared_ptr<I2CDevice>>> bus_chain, const float dt, uint16_t bus_num);
void signal_callback_handler(int signum);


int main(int argc, char *argv[]) {
    signal(SIGINT, signal_callback_handler);

    i2c_dev = pigpio_start(nullptr, nullptr);
    if(i2c_dev < 0) {
        cerr << "cannot connect to pigpiod!" << endl;
        exit(EXIT_FAILURE);
    }

    handle_yaml_arguments(argc, argv);
    if(verbose) cout << "argparse done!" << endl;

    is_running = true;
    if(plot_udp) {
        if(verbose) cout << "setting up UDP thread... " << endl;
        threads[0] = std::thread([&]() {sock_udp->loop();});
    }

    if(verbose) cout << "setting up bus threads..." << endl;
    for(auto&& d: devs) {
        cout << "    [" << d.first << "]: " << d.second->size() << "-- " << devs[d.first]->size() << endl;
    }

    for(auto&& bus_pair: devs) {
        auto bus_num = bus_pair.first;
        auto dev_vec = bus_pair.second;

        if(verbose) cout << "    setting up thread for bus #" << bus_num << ", " << typeid(dev_vec).name() << " " << dev_vec->size() << " @" << dev_vec->data() << endl;
        threads[bus_num] = std::thread([=]() {acquire(dev_vec, dt, bus_num);});
    }

    if(verbose) cout << "Configuration done. Waiting for threads to join ..." << endl;
    for(auto&& th_it : threads) {
        th_it.second.join();
    }

    if(store_to_shm) {
        shm_body_acc->close(false);
        shm_body_pos->close(false);
        shm_body_w->close(false);
        shm_joint_q->close(false);
    }

    // cleaning up before close the app.
    cout << "Cleaning up...";
    pigpio_stop(i2c_dev);
    cout << " Done." << endl;

    return EXIT_SUCCESS;
}

void acquire(shared_ptr<vector<std::shared_ptr<I2CDevice>>> bus_chain, const float dt, uint16_t bus_num) {
    chrono::duration<double> elapsed;

    while(is_running) {
        for(auto&& d: *bus_chain) {
            if(plot_udp) {
                if(verbose) cout << "[acquire] time elapsed ... ";
                elapsed = chrono::system_clock::now() - sock_udp->t0;
                if(verbose) cout << elapsed.count() << endl;
            }
    
            if(auto enc = dynamic_pointer_cast<AM4096>(d)) { // != nullptr
                enc->update();
                if(plot_udp) sock_udp->data["enc"][joint_names[enc->id]] = enc->pos;
                if(verbose) cout << "[acquire::AM4096:" << dec2hex(enc->id) << "] pos[" << enc->valid << "] :" << enc->pos << endl;
                if(store_to_shm) {
                    double shm_q[7]; // 0:JTID_NK, 1:JTID_LH, 2:JTID_LK, 3:JTID_LA, 4:JTID_RH, 5:JTID_RK, 6:JTID_RA
                    shm_joint_q_lock.lock();
                    shm_joint_q->get(shm_q);
                    if(0x11 == enc->id) { // JTID_LH
                        shm_q[1] = enc->pos;
                    } else if (0x12 == enc->id) { // JTID_LK
                        shm_q[2] = enc->pos;
                    } else if (0x13 == enc->id) { // JTID_RH
                        shm_q[4] = enc->pos;
                    } else if (0x14 == enc->id) { // JTID_RK
                        shm_q[5] = enc->pos;
                    }
                    shm_joint_q->set(shm_q);
                    shm_joint_q_lock.unlock();
                }
            } else if (auto imu = dynamic_pointer_cast<ISM330DHCX>(d)) { // != nullptr
                imu->update();
                if(plot_udp) {
                    // sock_udp->data["imu"]["t_imu"] = elapsed.count();
                    sock_udp->data["imu"]["gyro_x"] = imu->gyro[0];
                    sock_udp->data["imu"]["gyro_y"] = imu->gyro[1];
                    sock_udp->data["imu"]["gyro_z"] = imu->gyro[2];
                    sock_udp->data["imu"]["acc_x"] = imu->acc[0];
                    sock_udp->data["imu"]["acc_y"] = imu->acc[1];
                    sock_udp->data["imu"]["acc_z"] = imu->acc[2];
                }
                if(verbose) {
                    cout << "[acquire::ISM330DHCX:" << dec2hex(imu->id) << "] acc:(" << imu->acc[0] << ", " << imu->acc[1] << ", " << imu->acc[2] << ")";
                    cout << ", gyro: (" << imu->gyro[0] << ", " << imu->gyro[1] << ", " << imu->gyro[2] << ")";
                    cout << ", temp: " << imu->temperature << "\t" << endl;
                }
                if(store_to_shm) {
                    double shm_acc[3], shm_w[3];
                    shm_acc[0] = imu->acc[0];
                    shm_acc[1] = imu->acc[1];
                    shm_acc[2] = imu->acc[2];
                    shm_body_acc->set(shm_acc);

                    shm_w[0] = imu->gyro[0];
                    shm_w[1] = imu->gyro[1];
                    shm_w[2] = imu->gyro[2];
                    shm_body_w->set(shm_w);
                }
            } else if(auto imu = dynamic_pointer_cast<BNO08X>(d)) {
                imu->update();
                if(plot_udp) {
                    // sock_udp->data["imu"]["t_imu"] = elapsed.count();
                    if(imu->periodGyro > 0) {
                        sock_udp->data["imu"]["gyro_x"] = imu->gyro[0];
                        sock_udp->data["imu"]["gyro_y"] = imu->gyro[1];
                        sock_udp->data["imu"]["gyro_z"] = imu->gyro[2];
                    }
                    if(imu->periodAcc > 0) {
                        sock_udp->data["imu"]["acc_x"] = imu->acc[0];
                        sock_udp->data["imu"]["acc_y"] = imu->acc[1];
                        sock_udp->data["imu"]["acc_z"] = imu->acc[2];
                    }
                    if(imu->periodOri > 0) {
                        sock_udp->data["imu"]["qt_w"] = imu->qt[0];
                        sock_udp->data["imu"]["qt_x"] = imu->qt[1];
                        sock_udp->data["imu"]["qt_y"] = imu->qt[2];
                        sock_udp->data["imu"]["qt_z"] = imu->qt[3];
                    }
                    if(imu->periodGravity > 0) {
                        sock_udp->data["imu"]["g_x"] = imu->g[0];
                        sock_udp->data["imu"]["g_y"] = imu->g[1];
                        sock_udp->data["imu"]["g_z"] = imu->g[2];
                    }
                    if(imu->periodMag > 0 ) {
                        sock_udp->data["imu"]["mag_x"] = imu->mag[0];
                        sock_udp->data["imu"]["mag_y"] = imu->mag[1];
                        sock_udp->data["imu"]["mag_z"] = imu->mag[2];
                    }
                }
                if(verbose) {
                    cout << "[acquire::BNO08X:" << dec2hex(imu->id) << "] acc:(" << imu->acc[0] << ", " << imu->acc[1] << ", " << imu->acc[2] << ")";
                    cout << ", gyro: (" << imu->gyro[0] << ", " << imu->gyro[1] << ", " << imu->gyro[2] << ")" << endl;
                }
                if(store_to_shm) {
                    double shm_acc[3], shm_w[3], shm_qt[4];
                    if(imu->periodAcc > 0) {
                        shm_acc[0] = imu->acc[0];
                        shm_acc[1] = imu->acc[1];
                        shm_acc[2] = imu->acc[2];
                        shm_body_acc->set(shm_acc);
                    }

                    if(imu->periodGyro > 0) {
                        shm_w[0] = imu->gyro[0];
                        shm_w[1] = imu->gyro[1];
                        shm_w[2] = imu->gyro[2];
                        shm_body_w->set(shm_w);
                    }

                    if(imu->periodOri > 0) {
                        shm_qt[0] = imu->qt[0];
                        shm_qt[1] = imu->qt[1];
                        shm_qt[2] = imu->qt[2];
                        shm_qt[3] = imu->qt[3];
                        shm_body_qt->set(shm_qt);
                    }
                }
            }else if (auto tof = dynamic_pointer_cast<VL53LX>(d)) {
                static int _tof_skip_idx = 0;
                if(((++_tof_skip_idx) % 10) != 1) continue;
                tof->update();
                if(plot_udp) {
                    sock_udp->data["tof"]["range"] = tof->range;
                }
                
                bool touted = tof->timeoutOccurred();
                if(verbose) {
                    cout << "[acquire::VL53LX:" << dec2hex(tof->id) << "] range: " << tof->range << " m " << endl;
                    if (touted) cout << "[acquire::VL53LX] timeout last time" << endl;
                }

                if(store_to_shm) {
                    double shm_tof[3];
                    shm_body_pos->get(shm_tof);
                    shm_tof[2] = tof->range;
                    shm_body_pos->set(shm_tof);
                }
            } else {
                cout << "[acquire::WrongType] ... " << endl;
            }

            if (dt > 0) usleep(dt * 1e6);
        }
    }

}

void handle_yaml_arguments(int argc, char* argv[]) {
    auto args = argh::parser(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);

    if(args({"-h", "--help"})) {
        cout << "Usage: calibrate_encoder [--yaml PATH=sensors.yaml]" << endl;
        exit(EXIT_FAILURE);
    }

    const string filename = (args({"-y", "--yaml"})) ? args({"-y", "--yaml"}).str() : "./sensors.yaml";
    YAML::Node config = YAML::LoadFile(filename);

    auto ac_config = config["acquisition"];
    verbose = ac_config["verbose"].as<bool>();
    cout << "verbose: " << verbose << endl;

    dt = ac_config["dt"].as<float>();
    if(verbose) cout << "dt: " << dt << endl;

    if(config["shared_memory"]) {
        auto shm_config = config["shared_memory"];
        store_to_shm = shm_config["enable"].as<bool>();
        cout << "Store data into shared_memory: " << store_to_shm;

        if (store_to_shm) {
            shm_namespace = shm_config["namespace"].as<string>();
            cout << " -- " << shm_namespace;
        }
        cout << endl;
            
    } else {
        store_to_shm = false;
    }

    if(store_to_shm) {
        shm_body_pos = make_shared<Shmem3DVector>(("/" + shm_namespace + "_BODY_STATE_p").c_str(), true);
        shm_body_acc = make_shared<Shmem3DVector>(("/" + shm_namespace + "_BODY_STATE_a").c_str(), true);
        shm_body_qt = make_shared<Shmem3DVector>(("/" + shm_namespace + "_BODY_STATE_qt").c_str(), true);
        shm_body_w = make_shared<Shmem3DVector>(("/" + shm_namespace + "_BODY_STATE_w").c_str(), true);
        shm_joint_q = make_shared<ShmemJointVector>(("/" + shm_namespace + "_JOINT_STATE_q").c_str(), true);
        
        if(shm_body_pos->addr == nullptr || shm_body_acc->addr == nullptr || shm_body_w->addr == nullptr || shm_joint_q->addr == nullptr) {
            store_to_shm = false;
            shm_body_pos = nullptr;
            shm_body_acc = nullptr;
            shm_body_qt = nullptr;
            shm_body_w = nullptr;
            shm_joint_q = nullptr;
            cout << "Cannot open/initialize Shared Memory!" << endl;
        }
    }

    plot_udp = config["plot"]["enable"].as<bool>();
    if(plot_udp) {
        const string ip = config["plot"]["ip"].as<string>();
        const uint16_t port = config["plot"]["port"].as<uint16_t>();
        std::cout << "UDP Plotting enabled! - " << ip << ":" << port << std::endl;
        sock_udp = std::make_unique<PlotUDPSocket>(ip, port, dt);
    }

    auto i2c_devs = config["i2c_devices"];
    uint32_t bus_num;
    
    for(auto&& bus_it = i2c_devs.begin(); bus_it != i2c_devs.end(); ++bus_it) {
        auto bus = vector<std::shared_ptr<I2CDevice>>();
        bus_num = bus_it->first.as<uint32_t>();
        for(auto&& dev_it = bus_it->second.begin(); dev_it != bus_it->second.end(); ++dev_it) {
            uint32_t dev_id = dev_it->first.as<uint32_t>();
            string dev_name = dev_it->second["name"].as<string>(); // dev label
            string dev_type = dev_it->second["type"].as<string>(); // dev class name

            joint_names[dev_id] = dev_name;
            
            int ping_res = -1;
            if ("AM4096" == dev_type) {
                bus.push_back(make_shared<AM4096>(i2c_dev, bus_num, dev_id)); // move the ownership to local bus.
                ping_res = bus.back()->ping();
            } else if ("ISM330DHCX" == dev_type) {
                bus.push_back(make_shared<ISM330DHCX>(i2c_dev, bus_num, dev_id));
                ping_res = bus.back()->ping();
            } else if ("BNO08X" == dev_type) {
                long periodAcc     = 5000; 
                long periodGyro    = 5000;
                long periodOri     = 5000;
                long periodGravity = 5000;
                long periodMag     = 5000;
                if(dev_it->second["periods"]) {
                    auto T_config = dev_it->second["periods"];
                    periodOri = T_config["orientation"].as<long>();
                    periodAcc = T_config["accelerometer"].as<long>();
                    periodGyro = T_config["gyroscope"].as<long>();
                    periodGravity = T_config["gravity"].as<long>();
                    periodMag = T_config["magnetometer"].as<long>();
                }

                bus.push_back(make_shared<BNO08X>(i2c_dev, bus_num, dev_id, periodOri, periodAcc, periodGyro, periodGravity, periodMag));
                ping_res = bus.back()->ping();
            } else if ("VL53LX" == dev_type) {
                auto tof = make_shared<VL53LX>(i2c_dev, bus_num, dev_id);
                if(dev_it->second["timeout"])           tof->setTimeout(dev_it->second["timeout"].as<uint16_t>());
                if(dev_it->second["rate_limit"])        tof->setSignalRateLimit(dev_it->second["rate_limit"].as<float>());
                if(dev_it->second["period_prerange"])   tof->setVcselPulsePeriod(VL53LX::VcselPeriodPreRange, dev_it->second["period_prerange"].as<uint16_t>());
                if(dev_it->second["period_finalrange"]) tof->setVcselPulsePeriod(VL53LX::VcselPeriodFinalRange, dev_it->second["period_finalrange"].as<uint16_t>());
                if(dev_it->second["timing_budget"])     tof->setMeasurementTimingBudget(dev_it->second["timing_budget"].as<uint32_t>());
                // continuous = dev_it->second["continuous"].as<bool>();

                ping_res = tof->init()?1:0;
                if(ping_res > 0) {
                    bus.push_back(move(tof)); // not necessary tho.
                    // bus.push_back(make_shared<VL53LX>(tof));
                } else {
                    cout << " -- error: Cannot initialize ToF sensor!" << endl;
                }
            }

            if(verbose) {
                cout << "[YAMLHandler::"<< dev_type << "] " << dev_type << ":" << dec2hex(bus.back()->id) << " configured @" << bus.back() << " ";
                if(ping_res > 0) { cout << "and added to the chain #" << bus_num << "@" << bus.data(); } else { cout << " -- error."; }
                cout << endl;
            }
            if (ping_res < 1)  {
                if(dev_type != "VL53LX") bus.pop_back();
            }
        
        }

        if(bus.size() > 0)
            devs.insert(make_pair(bus_num, make_shared<vector<std::shared_ptr<I2CDevice>>>(bus))); // bus' ownership is not moved.
    }

}

void signal_callback_handler(int signum) {
    cout << "Caught signal " << signum << endl;
    if(plot_udp) sock_udp->is_running = false;
    is_running = false;
}