//
// Created by Hosik Chae on 23. 2. 8.
//

#include <iostream>
#include <unistd.h>
#include <cstdlib> // exit
#include <cmath> //NAN
#include <chrono>
#include <memory>
#include <pigpiod_if2.h>
#include <stdlib.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "argh.h"
#include "json.hpp"
#include "encoder.h"

#define IP "127.0.0.1"
#define PORT 9870
#define BUFF_SIZE 100

int main(int argc, char *argv[]) {
    auto args = argh::parser(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);

    string usage = "Usage: calibrate_encoder BUS I2C_ADDR N DT [--plot HOST_IP]";
    if(args.size() != 5) {
        std::cout << usage << std::endl;
    }

    // =======================================
    // UDP socket communication
    bool plot_udp = false;
    if(args({"-p", "--plot"})) {
        std::cout << "UDP Plotting enabled!" << std::endl;
        plot_udp = true;
    }

    int sock;
    sockaddr_in sockAddr;
    char buf[BUFF_SIZE+1];
    nlohmann::json_abi_v3_11_2::json j;
//    double t0_plot, t_plot;
    if(plot_udp) {
        sock = socket(PF_INET, SOCK_DGRAM, 0);
        if (sock == -1) {
            std::cout << "Failed to create an UDP socket." << std::endl;
            exit(0);
        }

        bzero(&sockAddr, sizeof(sockAddr)); // memset(&addr, 0, sizeof(addr));
        sockAddr.sin_family = AF_INET;
        sockAddr.sin_port = htons(PORT);             // port number
        sockAddr.sin_addr.s_addr = inet_addr(IP); // ip
    }
    // =======================================

    if(!args(1)) {
        std::cout << "BUS argument has to be given! -- " << usage << std::endl;
        return EXIT_FAILURE;
    } else if(!args(2)) {
        std::cout << "I2C_ADDR_HEX argument has to be given! -- " << usage << std::endl;
        return EXIT_FAILURE;
    } else if(!args(3)) {
        std::cout << "N > 0 argument has to be given! -- " << usage << std::endl;
        return EXIT_FAILURE;
    }

    uint16_t bus = atoi(args[1].c_str());
    uint16_t addr = -1;

    if(args[2].rfind("0x", 0) == 0 || args[2].rfind("0X", 0) == 0) {
        // When I2C_ADDR_HEX is in HEXADECIMAL form
        addr = (int)strtol(args[2].c_str(), nullptr, 0);
    } else {
        addr = atoi(args[2].c_str());
    }

    int i2c_dev = pigpio_start(nullptr, nullptr);
    if(i2c_dev < 0) {
        cerr << "cannot connect to pigpiod!" << endl;
        exit(EXIT_FAILURE);
    }

    uint64_t N = atoi(args[3].c_str());
    float dt = atof(args[4].c_str());
    float t = 0.0;
    auto enc = std::make_unique<AM4096>(i2c_dev, bus, addr);
    
    for(size_t ndx = 0; ndx < N; ndx++) {
        enc->update();
        t = ndx * dt;
        std::cout << "[AM4096(" << enc->id << ")] (rawPos, pos[valid], magnet): ";
        std::cout << enc->rawPos << ", " << enc->pos << "[" << enc->valid << "]" << std::endl ;

        j = {
                {"timestamp", t},
                {"enc", { {dec2hex(enc->id), sin(2 * M_PI * t)} } },
        };


        bzero(buf, BUFF_SIZE);
        memcpy(buf, j.dump().c_str(), strlen(j.dump().c_str()));
        sendto(sock, buf, strlen(buf), 0, (sockaddr*)&sockAddr, sizeof(sockAddr));


        usleep(dt * 1e6);
    }

    if(plot_udp) {
        if(sock) close(sock);
    }
    // cleaning up before close the app.
    cout << "Cleaning up...";
    pigpio_stop(i2c_dev);
    cout << " Done." << endl;

    return EXIT_SUCCESS;
}