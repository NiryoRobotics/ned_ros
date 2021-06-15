/*
    dxl_debug.cpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//ros
#include <ros/ros.h>

//c++
#include <boost/program_options.hpp>
#include <iostream>
#include <string>
#include <vector>

//niryo
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dxl_debug_tools/dxl_tools.h"

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
#elifdef __aarch64__
    #define DEFAULT_PORT "/dev/ttyAMA0"
#else
    #define DEFAULT_PORT ""
#endif


int main (int argc, char **argv)
{
    ros::init(argc, argv, "dxl_debug_tools");

    ROS_DEBUG("Launching dxl_debug_tools");

    try {
        // Get args
        po::options_description description("Options");
        description.add_options()
            ("help,h", "Print help message")
            ("baudrate,b", po::value<int>()->default_value(1000000), "Baud rate")
            ("port,p", po::value<std::string>()->default_value(DEFAULT_PORT), "Set port")
            ("id,i", po::value<int>()->default_value(0), "Dxl motor ID")
            ("scan", "Scan all Dxl motors on the bus")
            ("ping", "ping specific ID")
            ("get-register", po::value<int>()->default_value(-1), "Get a value from a register (arg: reg_addr)")
            ("size", po::value<int>()->default_value(1), "Size (for get-register only)")
            ("set-register", po::value<std::vector<int>>(), "Set a value to a register (args: reg_addr, value, size)");

        po::positional_options_description p;
        p.add("set-register", -1);
        po::variables_map vars;
        po::store(po::command_line_parser(argc, argv).options(description).positional(p).run(), vars);
        po::notify(vars);

        // Display usage if no args or --help
        if (argc == 1 || vars.count("help")) {
            std::cout << description << "\n";
            return 0;
        }

        // --------   other commands

        int baudrate = vars["baudrate"].as<int>();
        std::string serial_port = vars["port"].as<std::string>();
        uint8_t id = vars["id"].as<uint8_t>();

        std::cout << "Using baudrate: " << baudrate << ", port: " << serial_port << "\n";
        std::cout << "Dxl ID: " << static_cast<int>(id) << "\n";

        // Setup Dxl communication
        std::shared_ptr<dynamixel::PortHandler> portHandler(dynamixel::PortHandler::getPortHandler(serial_port.c_str()));
        std::shared_ptr<dynamixel::PacketHandler> packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));

        dxl_debug_tools::DxlTools dxlTools(portHandler, packetHandler);

        if (-1 != dxlTools.setupDxlBus(baudrate))
        {
            int dxl_comm_result = COMM_TX_FAIL;

            // Execute action from args
            if (vars.count("scan")) //scan
            {
                printf("--> SCAN Dxl bus\n");
                dxlTools.broadcastPing();
            }
            else if (vars.count("ping")) //ping
            {
                if (0 == id) {
                    printf("Ping: you need to give an ID! (--id)\n");
                }
                else {
                    printf("--> PING Motor (ID: %d)\n", id);
                    dxlTools.ping(id);
                }
            }
            else if (vars.count("set-register")) //set-register
            {
                std::vector<int> params = vars["set-register"].as<std::vector<int>>();
                if (params.size() != 3) {
                    printf("ERROR: set-register needs 3 arguments (reg_addr, value, size)\n");
                }
                else {
                    uint8_t addr = static_cast<uint8_t>(params.at(0));
                    uint32_t value = static_cast<uint32_t>(params.at(1));
                    uint8_t size = static_cast<uint8_t>(params.at(2));

                    printf("--> SET REGISTER for Motor (ID:%d)\n", id);
                    printf("Register address: %d, Value: %d, Size (bytes): %d\n", addr, value, size);

                    dxl_comm_result = dxlTools.setRegister(id, addr, value, size);

                    if (dxl_comm_result != COMM_SUCCESS)
                        printf("Failed to set register: %d\n", dxl_comm_result);
                    else
                        printf("Successfully sent register command\n");
                }
            }
            else if (vars.count("get-register")) // get-register
            {
                uint8_t addr = vars["get-register"].as<uint8_t>();
                uint32_t value = 0;
                uint8_t size = vars["size"].as<uint8_t>();

                printf("--> GET REGISTER for Motor (ID:%d)\n", id);
                printf("Register address: %d, Size (bytes): %d\n", addr, size);

                dxl_comm_result = dxlTools.getRegister(id, addr, value, size);

                if (dxl_comm_result != COMM_SUCCESS)
                    printf("Failed to get register: %d\n", dxl_comm_result);
                else
                    printf("Retrieved value at address %d : %d\n", addr, value);
            }
            else //unknown command
            {
                std::cout << description << "\n";
            }

            // close port before exit
            dxlTools.closePort();
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
