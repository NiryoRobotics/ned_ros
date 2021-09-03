/*
    ttl_debug.cpp
    Copyright (C) 2018 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

// c++
#include <boost/program_options.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>     //for using the function sleep
#include <chrono>
#include <thread>

// niryo
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ttl_debug_tools/ttl_tools.h"

#define PROTOCOL_VERSION 2.0

// params:
// - baud_rate (optional)
// - serial port (optional)
// - motor ID

// available actions:
// - scan
// - ping specific ID
// - set register

namespace po = boost::program_options;

#ifdef __arm
    #define DEFAULT_PORT "/dev/serial0"
#elif __aarch64__
    #define DEFAULT_PORT "/dev/ttyAMA0"
#else
    #define DEFAULT_PORT ""
#endif

int main(int argc, char **argv)
{
    try
    {
        // Get args
        po::options_description description("Options");
        description.add_options()
            ("help,h", "Print help message")
            ("baudrate,b", po::value<int>()->default_value(1000000), "Baud rate")
            ("port,p", po::value<std::string>()->default_value(DEFAULT_PORT), "Set port")
            ("id,i", po::value<int>()->default_value(0), "Motor ID")
            ("scan", "Scan all motors on the TTL bus")
            ("ping", "ping specific ID")
            ("get-register", po::value<int>()->default_value(-1), "Get a value from a register (arg: reg_addr)")
            ("size", po::value<int>()->default_value(1), "Size (for get-register only)")
            ("calibrate", "calibrate joints")
            ("set-register", po::value<std::vector<int>>(), "Set a value to a register (args: reg_addr, value, size)");

        po::positional_options_description p;
        p.add("set-register", -1);
        po::variables_map vars;
        po::store(po::command_line_parser(argc, argv).options(description).positional(p).run(), vars);
        po::notify(vars);

        // Display usage if no args or --help
        if (argc == 1 || vars.count("help"))
        {
            std::cout << description << "\n";
            return 0;
        }

        // --------   other commands

        int baudrate = vars["baudrate"].as<int>();
        std::string serial_port = vars["port"].as<std::string>();
        int id = vars["id"].as<int>();

        std::cout << "Using baudrate: " << baudrate << ", port: " << serial_port << "\n";
        std::cout << "Motor ID: " << id << "\n";

        // Setup TTL communication
        std::shared_ptr<dynamixel::PortHandler> portHandler(
                    dynamixel::PortHandler::getPortHandler(serial_port.c_str()));

        std::shared_ptr<dynamixel::PacketHandler> packetHandler(
                    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));

        ttl_debug_tools::TtlTools ttlTools(portHandler, packetHandler);

        if (-1 != ttlTools.setupBus(baudrate))
        {
            int comm_result = COMM_TX_FAIL;

            // Execute action from args
            if (vars.count("scan"))  // scan
            {
                printf("--> SCAN TTL bus\n");
                ttlTools.broadcastPing();
            }
            else if (vars.count("calibrate"))  // calibrate
            {
                printf("--> calibrate joints\n");
                ttlTools.setRegister(3, 149, 1, 1);
                ttlTools.setRegister(2, 147, 0, 1);
                ttlTools.setRegister(3, 147, 0, 1);
                ttlTools.setRegister(2, 147, 0, 1);
            }
            else if (vars.count("ping"))  // ping
            {
                if (0 == id)
                {
                    printf("Ping: you need to give an ID! (--id)\n");
                }
                else
                {
                    printf("--> PING Motor (ID: %d)\n", id);
                    ttlTools.ping(id);
                }
            }
            else if (vars.count("set-register"))  // set-register
            {
                std::vector<int> params = vars["set-register"].as<std::vector<int>>();
                if (params.size() != 3)
                {
                    printf("ERROR: set-register needs 3 arguments (reg_addr, value, size)\n");
                }
                else
                {
                    uint8_t addr = static_cast<uint8_t>(params.at(0));
                    uint32_t value = static_cast<uint32_t>(params.at(1));
                    uint8_t size = static_cast<uint8_t>(params.at(2));

                    printf("--> SET REGISTER for Motor (ID:%d)\n", id);
                    printf("Register address: %d, Value: %d, Size (bytes): %d\n", addr, value, size);

                        comm_result = ttlTools.setRegister(static_cast<uint8_t>(id), addr, value, size);

                    if (comm_result != COMM_SUCCESS)
                        printf("Failed to set register: %d\n", comm_result);
                    else
                        printf("Successfully sent register command\n");
                }
            }
            else if (vars.count("get-register"))  // get-register
            {
                uint8_t addr = static_cast<uint8_t>(vars["get-register"].as<int>());
                uint32_t value = 0;
                uint8_t size = static_cast<uint8_t>(vars["size"].as<int>());

                printf("--> GET REGISTER for Motor (ID:%d)\n", id);
                printf("Register address: %d, Size (bytes): %d\n", addr, size);

                comm_result = ttlTools.getRegister(static_cast<uint8_t>(id), addr, value, size);

                if (comm_result != COMM_SUCCESS)
                    printf("Failed to get register: %d\n", comm_result);
                else
                    printf("Retrieved value at address %d : %d\n", addr, value);
            }
            else  // unknown command
            {
                std::cout << description << "\n";
            }

            // close port before exit
            ttlTools.closePort();
            return 0;
        }

        return -1;
    }
    catch(po::error& e)
    {
        std::cout << e.what() << "\n";
        return 0;
    }
}
