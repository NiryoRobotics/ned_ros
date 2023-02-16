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
#include <chrono>  // NOLINT [build/c++11]
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>  // NOLINT [build/c++11]
#include <vector>

#include "mcp_can_rpi/mcp_can_rpi.h"

// niryo
#include "can_debug_tools/can_tools.hpp"

namespace po = boost::program_options;

// CC : just for example
#ifdef NIRYO_ONE
#pragma message "One compilation"
#endif

#ifdef NIRYO_NED
#pragma message "Ned compilation"
#endif

#ifdef NIRYO_NED2
#pragma message "Ned 2 compilation"
#endif

/*
 * @brief handleUserInput
 * @param argc
 * @param argv
 * @return
 */
int handleUserInput(int argc, char **argv)
{
    try
    {
        // Get args#pragma message ( "C Preprocessor got here!" )
        po::options_description description("Options");
        description.add_options()("help,h", "Print help message")("channel,c", po::value<int>()->default_value(0), "Spi Channel")(
            "baudrate,b", po::value<int>()->default_value(1000000), "Baud rate")("gpio,g", po::value<int>()->default_value(25), "gpio can interrupt")(
            "freq,f", po::value<double>()->default_value(100.0), "data check frequency in Hz (must be > 0)")("dump", "Dump any data from bus");

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
        int spi_channel = vars["channel"].as<int>();
        int spi_baudrate = vars["baudrate"].as<int>();
        int gpio_can_interrupt = vars["gpio"].as<int>();

        std::cout << "Using channel: " << spi_channel << ", "
                  << "Using baudrate: " << spi_baudrate << ", "
                  << "Using gpio: " << gpio_can_interrupt << "\n";

        // Setup TTL communication
        auto mcp_can = std::make_shared<mcp_can_rpi::MCP_CAN>(spi_channel, spi_baudrate, static_cast<uint8_t>(gpio_can_interrupt));

        can_debug_tools::CanTools canTools(mcp_can);

        if (CAN_OK == canTools.setupCommunication())
        {
            // Execute action from args
            if (vars.count("dump"))  // dump data
            {
                double check_data_freq = vars["freq"].as<double>();
                if (0.0 != check_data_freq)
                {
                    printf("--> Dumping CAN bus.\n");
                    canTools.startDump(check_data_freq);
                    std::cout << "Press Enter to Exit" << std::endl;
                    std::cin.get();
                    return 0;
                }

                std::cout << description << "\n";
                return -1;
            }
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
