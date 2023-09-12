//
// Created by Hosik Chae on 23. 2. 8.
//

#include <iostream>
#include <unistd.h>
#include <cstdlib> // exit
#include <cmath> //NAN
#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>
#include <pigpiod_if2.h>
#include <stdlib.h>

#include <yaml-cpp/yaml.h>
#include "argh.h"
#include "encoder.h"

using namespace std;
using DevDict = unordered_map<string, string>;
using BusDict = unordered_map<uint32_t, DevDict>;

int i2c_dev;
bool verbose = false;

void calibrate_by_yaml(const argh::parser args);
void calibrate_by_cml(const argh::parser args);

string usage = "Usage: calibrate_encoder --yaml YAML \t or\n       calibrate_encoder BUS I2C_ADDR [ROT_DIR], ROT_DIR is (0:pos axis, 1:neg axis, -1: reset offset)";

int main(int argc, char *argv[]) {
    auto args = argh::parser(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);

    if(args({"-h", "--help"})) {
        cout << usage << endl;
        exit(EXIT_FAILURE);
    }

    int i2c_dev = pigpio_start(nullptr, nullptr);
    if(i2c_dev < 0) {
        cerr << "cannot connect to pigpiod!" << endl;
        exit(EXIT_FAILURE);
    }

    if(args({"-y", "--yaml"})) {
        calibrate_by_yaml(args);
    } else {
        calibrate_by_cml(args);
    }

    

    // cleaning up before close the app.
    cout << "Cleaning up...";
    pigpio_stop(i2c_dev);
    cout << " Done." << endl;

    return EXIT_SUCCESS;
}

void calibrate_by_cml(const argh::parser args) {
    if (!(args.size() == 3 || args.size() == 4)) {
        std::cout << usage << std::endl;
        exit(EXIT_FAILURE);
    } 

    if(!args(1)) {
        std::cout << "BUS argument has to be given! -- \n" << usage << std::endl;
        exit(EXIT_FAILURE);
    } else if(!args(2)) {
        std::cout << "I2C_ADDR_HEX argument has to be given! -- \n" << usage << std::endl;
        exit(EXIT_FAILURE);
    }

    uint16_t bus = atoi(args[1].c_str());
    uint16_t addr = -1;
    uint16_t axis = -1;
    bool reset_offset = false;

    if(args[2].rfind("0x", 0) == 0 || args[2].rfind("0X", 0) == 0) {
        // When I2C_ADDR_HEX is in HEXADECIMAL form
        addr = (int)strtol(args[2].c_str(), nullptr, 0);
    } else {
        addr = atoi(args[2].c_str());
    }

    auto enc = std::make_unique<AM4096>(i2c_dev, bus, addr);

    if(args(3)) {
        if(args[3].rfind("x", 0) == 0 || args[3].rfind("X", 0) == 0) {
            // reset encoder offset
            cout << "resetting offset!" << endl;
            reset_offset = true;
            enc->getRotationSign(true);
        } else {
            // change axis
            axis = atoi(args[3].c_str());
            enc->setRotationSign((bool)axis);
        }
    }

    uint32_t res;
    if(reset_offset) {
        res = enc->calibrateOffset(0, true);
    } else {
        res = enc->calibrateOffset(true); // call update() internally
    }

    std::cout << "result: " << res << std::endl;

}

void calibrate_by_yaml(const argh::parser args) {
    const string filename = args({"-y", "--yaml"}).str().size() > 0 ? args({"-y", "--yaml"}).str() : "./sensors.yaml";
    YAML::Node config = YAML::LoadFile(filename);

    verbose = config["calibration"]["verbose"].as<bool>();
    cout << "verbose: " << verbose << endl;

    auto i2c_devs = config["i2c_devices"];
    uint32_t bus_num;
    
    for(auto&& bus_it = i2c_devs.begin(); bus_it != i2c_devs.end(); ++bus_it) {
        bus_num = bus_it->first.as<uint32_t>();
        for(auto&& dev_it = bus_it->second.begin(); dev_it != bus_it->second.end(); ++dev_it) {
            uint32_t dev_id = dev_it->first.as<uint32_t>();
            string dev_type = dev_it->second["type"].as<string>(); // dev class name

            if ("AM4096" == dev_type) {
                int16_t enc_offset = dev_it->second["offset"].as<int16_t>();
                bool enc_axis = (dev_it->second["axis"].as<int16_t>() > 0);
                auto enc = make_shared<AM4096>(i2c_dev, bus_num, dev_id);

                if (enc->ping() < 1) {
                    if(verbose) cout << "Encdoer(" << dec2hex(dev_id) << ") does not respond... Skip!";
                    continue;
                }

                // since encoder rotation axis affects the angular calibration, rotation setting has to come prior to calibration.
                enc->setRotationSign(enc_axis, verbose); // if verbose, printing the current axis is included

                if(enc_offset < 0) {
                    enc->calibrateOffset(verbose); // if verbose, printing the current offset is included
                } else {
                    enc->calibrateOffset(enc_offset, verbose);
                }

                cout << "------------------------------------------------" << endl;
            } else if ("ISM330DHCX" == dev_type) {}

        }
    }

}
