#include <iostream>
#include <string>
#include <stdlib.h>

#include <pigpiod_if2.h>
#include "argh.h"

#include "encoder.h"

using namespace std;

int main(int argc, char* argv[]) {
    auto args = argh::parser(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);
    const string usage = "[UpdateEncoderId] Usage: update_encoder_id BUS NEW_ID [OLD_ID=0]";

    uint8_t oldId = 0x00, newId = 0x00, bus=0x00;

    if(args({"-h", "--help"}) || (args.size() < 3) || (args.size() > 4)) {
        cout << usage << endl;
        return EXIT_FAILURE;
    }

    bus = atoi(args[1].c_str());

    if(args(2)) {
        if(args[2].rfind("0x", 0) == 0 || args[2].rfind("0X", 0) == 0) {
            // When I2C_ADDR_HEX is in HEXADECIMAL form
            newId = (int)strtol(args[2].c_str(), nullptr, 0);
        } else {
            newId = atoi(args[2].c_str());
        }
    }

    if(args(3)) {
        if(args[3].rfind("0x", 0) == 0 || args[3].rfind("0X", 0) == 0) {
            // When I2C_ADDR_HEX is in HEXADECIMAL form
            oldId = (int)strtol(args[3].c_str(), nullptr, 0);
        } else {
            oldId = atoi(args[3].c_str());
        }
    }

    


    int i2c_dev = pigpio_start(nullptr, nullptr);
    if(i2c_dev < 0) {
        cerr << "[UpdateEncoderId] cannot connect to pigpiod!" << endl;
        exit(EXIT_FAILURE);
    }

    auto encOld = AM4096(i2c_dev, bus, oldId);
    encOld.updateId(newId, 3.0, true);

    auto encNew = AM4096(i2c_dev, bus, newId);
    
    int ret = encNew.ping(true);
    if (ret > 0) {
        cout << "[UpdateEncoderId] Successfully updated!: " << encNew.chipID() << endl;
    } else {
        cout << "[UpdateEncoderId] Something went wrong ...!" << endl;
    }

    return EXIT_SUCCESS;
}