/*
    can_tools_unit_tests.cpp
    Copyright (C) 2020 Niryo
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

// Bring in my package's API, which is what I'm testing
#include "mcp_can_rpi/mcp_can_rpi.h"

#include "can_debug_tools/can_tools.hpp"

#include <memory>
#include <string>

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(CanDebugToolsTestSuite, testInit)
{
    int spi_channel = 0;
    int spi_baudrate = 1000000;
    int gpio_can_interrupt = 25;

    std::cout << "Using channel: " << spi_channel << ", "
              << "Using baudrate: " << spi_baudrate << ", "
              << "Using gpio: " << gpio_can_interrupt << "\n";

    // Setup TTL communication
    auto mcp_can = std::make_shared<mcp_can_rpi::MCP_CAN>(spi_channel, spi_baudrate, static_cast<uint8_t>(gpio_can_interrupt));

    can_debug_tools::CanTools canTools(mcp_can);

    ASSERT_NE(-1, canTools.setupCommunication());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
