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
#include "tof.h"
#include "plot_udp.h"

using namespace std;
bool verbose = false;
bool is_running = false;
bool plot_udp = false;

void handle_yaml_arguments(int argc, char* argv[]);
void signal_callback_handler(int signum);

shared_ptr<VL53LX> tof;
uint8_t periodPreRange, periodFinalRange;
uint32_t timingBudget;
float rateLimit;
bool continuous;

void test_single(shared_ptr<VL53LX> tof);
void test_continuous(shared_ptr<VL53LX> tof);

unique_ptr<PlotUDPSocket> sock_udp;

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_callback_handler);
    handle_yaml_arguments(argc, argv);
    
    int i2c_dev = pigpio_start(nullptr, nullptr);
    if(i2c_dev < 0) {
        cerr << "cannot connect to pigpiod!" << endl;
        exit(EXIT_FAILURE);
    }

    tof = make_shared<VL53LX>(i2c_dev, 2);    

    if(!tof->init()) {
        cout << "error in initializing tof" << endl;
    }
    tof->setSignalRateLimit(rateLimit);
    tof->setVcselPulsePeriod(VL53LX::VcselPeriodPreRange, periodPreRange);
    tof->setVcselPulsePeriod(VL53LX::VcselPeriodFinalRange, periodFinalRange);
    tof->setMeasurementTimingBudget(timingBudget);
    
    is_running = true;
    if(continuous) {
        test_continuous(tof);
    } else {
        test_single(tof);
    }
    

    return EXIT_SUCCESS;
}

void test_continuous(shared_ptr<VL53LX> tof) {
    // Start continuous back-to-back mode (take readings as
    // fast as possible).  To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).

    if(verbose) cout << "Continous Mode!" << endl;
    tof->startContinuous();

    uint16_t range_mm;
    while(is_running) {
        range_mm = tof->readRangeContinuousMillimeters();
        if(verbose) cout << range_mm << endl;
        if (tof->timeoutOccurred()) { 
            if(verbose) cout << "TimeOut!" << endl;; 
        }
    }
    tof->stopContinuous();
}

void test_single(shared_ptr<VL53LX> tof) {
    if(verbose) cout << "Single Mode!" << endl;
    uint16_t range_mm;
    while(is_running) {
        range_mm = tof->readRangeSingleMillimeters();
        if(verbose) cout << range_mm << endl;
        if(tof->timeoutOccurred()) {
            if(verbose) cout << "TimeOut!" << endl;
        }
    }
}

void handle_yaml_arguments(int argc, char* argv[]) {
    auto args = argh::parser(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);

    if(args({"-h", "--help"})) {
        cout << "Usage: echo_tof [--yaml PATH=sensors.yaml]" << endl;
        exit(EXIT_SUCCESS);
    }      

    const string filename = (args({"-y", "--yaml"})) ? args({"-y", "--yaml"}).str() : "./sensors.yaml";
    YAML::Node config = YAML::LoadFile(filename);

    verbose = config["verbose"].as<bool>();
    cout << "verbose: " << verbose << endl;

    auto tof_config = config["i2c_devices"]["2"]["0x29"];

    rateLimit = tof_config["rate_limit"].as<float>();
    periodPreRange = tof_config["period_prerange"].as<uint16_t>();
    periodFinalRange = tof_config["period_finalrange"].as<uint16_t>();
    timingBudget = tof_config["timing_budget"].as<uint32_t>();
    continuous = tof_config["continuous"].as<bool>();

    if(verbose) {
        cout << "Options: " << endl;
        cout << "  rateLimit(" << rateLimit << ")" << endl;
        cout << "  periodPreRange(" << unsigned(periodPreRange) << ")" << endl;
        cout << "  periodFinalRange(" << unsigned(periodFinalRange) << ")" << endl;
        cout << "  timingBudget(" << timingBudget << ")" << endl;
        cout << "  continuous(" << continuous << ")" << endl;
    }
}

void signal_callback_handler(int signum) {
    cout << "Caught signal " << signum << endl;
    if(plot_udp) sock_udp->is_running = false;
    is_running = false;
}