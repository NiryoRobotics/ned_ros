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
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <chrono>  // NOLINT
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>  // NOLINT
#include <vector>

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

#ifdef __arm__
#define DEFAULT_PORT "/dev/serial0"
#elif __aarch64__
#define DEFAULT_PORT "/dev/ttyAMA0"
#else
#define DEFAULT_PORT ""
#endif

/**
 * @brief handleUserInput
 * @param argc
 * @param argv
 * @return
 */
int handleUserInput(int argc, char **argv)
{
    try
    {
        // Get args
        po::options_description description("Options");
        description.add_options()("help,h", "Print help message")("baudrate,b", po::value<int>()->default_value(1000000), "Baud rate")(
            "port,p", po::value<std::string>()->default_value(DEFAULT_PORT), "Set port")("id,i", po::value<int>()->default_value(-1), "Motor ID")(
            "ids", po::value<std::vector<int>>()->multitoken(), "list of id for sync read or write")("scan", "Scan all motors on the TTL bus")("ping", "ping specific ID")(
            "get-register", po::value<int>(), "Get a value from a register (arg: reg_addr)")("size", po::value<int>()->default_value(1), "Size (for get-register only)")(
            "calibrate", "calibrate joints")("test", "a test movement")("set-register", po::value<std::vector<int>>()->multitoken(),
                                                                        "Set a value to a register (args: reg_addr, value, size)")(
            "set-registers", po::value<std::vector<int>>()->multitoken(), "Set the values to a register for multiples devices (args: reg_addr, size, values)")(
            "get-registers", po::value<int>()->multitoken(), "get the values of a register for multiples devices (arg: reg_addr)");

        // po::positional_options_description p;
        // p.add("set-register", 3);
        // p.add("set_registers", -1);
        // p.add("ids", -1);

        po::variables_map vars;
        po::store(po::parse_command_line(argc, argv, description), vars);
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
        std::vector<uint8_t> ids;
        if (vars.count("ids"))
        {
            std::vector<int> id_list = vars["ids"].as<std::vector<int>>();
            for (auto l_id : id_list)
            {
                ids.emplace_back(static_cast<uint8_t>(l_id));
            }
        }

        std::cout << "Using baudrate: " << baudrate << ", port: " << serial_port << "\n";

        // Setup TTL communication
        std::shared_ptr<dynamixel::PortHandler> portHandler(dynamixel::PortHandler::getPortHandler(serial_port.c_str()));

        std::shared_ptr<dynamixel::PacketHandler> packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));

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
                ttlTools.setRegister(2, 64, 0, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ttlTools.setRegister(3, 64, 0, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ttlTools.setRegister(4, 64, 0, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ttlTools.setRegister(3, 149, 1, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ttlTools.setRegister(2, 147, 0, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ttlTools.setRegister(3, 147, 0, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ttlTools.setRegister(4, 147, 0, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            else if (vars.count("test"))  // calibrate
            {
                uint32_t pos{};
                printf("--> Beginning Test\n");
                ttlTools.setRegister(2, 64, 1, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
                ttlTools.setRegister(2, 64, 0, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
                ttlTools.getRegister(2, 132, pos, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("pos %d\n", pos);
            }
            else if (-1 == id && ids.empty())
            {
                printf("Ping: you need to give an ID! (--id or --ids)\n");
            }
            else
            {
                if (vars.count("ping"))  // ping
                {
                    printf("--> PING Motor (ID: %d)\n", id);
                    ttlTools.ping(id);
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
                        auto addr = static_cast<uint16_t>(params.at(0));
                        auto value = static_cast<uint32_t>(params.at(1));
                        auto size = static_cast<uint8_t>(params.at(2));

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
                    uint16_t addr = static_cast<uint16_t>(vars["get-register"].as<int>());
                    uint32_t value = 0;
                    uint8_t size = static_cast<uint8_t>(vars["size"].as<int>());

                    printf("Register address: %d, Size (bytes): %d\n", addr, size);
                    comm_result = ttlTools.getRegister(static_cast<uint8_t>(id), addr, value, size);

                    if (comm_result != COMM_SUCCESS)
                        printf("Failed to get register: %d\n", comm_result);
                    else
                    {
                        printf("Retrieved value at address %d : %d\n", addr, value);
                    }
                }
                else if (vars.count("set-registers"))
                {
                    std::vector<int> params = vars["set-registers"].as<std::vector<int>>();
                    auto addr = static_cast<uint8_t>(params.at(0));
                    auto size = static_cast<uint8_t>(params.at(1));

                    std::stringstream ss;
                    ss << "register address : " << static_cast<int>(addr) << " size : " << static_cast<int>(size) << " values";
                    std::vector<uint32_t> values;
                    for (size_t i = 2; i < params.size(); i++)
                    {
                        values.emplace_back(static_cast<uint32_t>(params.at(i)));
                        ss << " " << params.at(i);
                    }

                    std::cout << ss.str() << std::endl;

                    comm_result = ttlTools.setRegisters(ids, addr, values, size);

                    if (comm_result != COMM_SUCCESS)
                        printf("Failed to set registers: %d\n", comm_result);
                    else
                        printf("Successfully sent registers command\n");
                }
                else if (vars.count("get-registers"))
                {
                    uint8_t addr = static_cast<uint8_t>(vars["get-registers"].as<int>());
                    std::vector<uint32_t> values;
                    uint8_t size = static_cast<uint8_t>(vars["size"].as<int>());

                    printf("Register address: %d, Size (bytes): %d\n", addr, size);
                    comm_result = ttlTools.getRegisters(ids, addr, values, size);

                    if (comm_result != COMM_SUCCESS)
                        printf("Failed to get registers: %d\n", comm_result);
                    else
                    {
                        std::stringstream ss;
                        for (unsigned int value : values)
                        {
                            ss << " " << value;
                        }
                        std::cout << "Retrieved values at address " << static_cast<int>(addr) << ":" << ss.str() << std::endl;
                    }
                }
                else  // unknown command
                {
                    std::cout << description << "\n";
                }
            }

            // close port before exit
            ttlTools.closePort();
            return 0;
        }

        return -1;
    }
    catch (po::error &e)
    {
        std::cout << e.what() << "\n";
        return 1;
    }
    catch (...)
    {
        std::cout << "Unknown error"
                  << "\n";
        return 1;
    }
}

int main(int argc, char **argv) { return handleUserInput(argc, argv); }
