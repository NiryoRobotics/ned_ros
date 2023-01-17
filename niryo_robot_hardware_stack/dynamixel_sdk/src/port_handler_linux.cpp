/*
 * port_handler_linux.cpp
 * Copyright (c) 2017, Niryo
 * All rights reserved.
 *
 * This library is an adaptation of dynamixel_sdk library for Raspberry Pi 4b with wiringPi
 * See license below
 */

/*******************************************************************************
 * Copyright 2017 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#if defined(__linux__)

#include <fcntl.h>
#include <string.h>
#include <time.h>

#if defined __arm__ || defined __aarch64__
#include <wiringPi.h>
#endif

#include "dynamixel_sdk/port_handler_linux.h"

#define LATENCY_TIMER                                                                                                                                                              \
    5  // msec (USB latency timer)
       // You should adjust the latency timer value. From the version Ubuntu 16.04.2, the default latency timer of the usb serial is '16 msec'.
       // When you are going to use sync / bulk read, the latency timer should be loosen.
       // the lower latency timer value, the faster communication speed.

// Note:
// You can check its value by:
// $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
//
// If you think that the communication is too slow, type following after plugging the usb in to change the latency timer
//
// Method 1. Type following (you should do this everytime when the usb once was plugged out or the connection was dropped)
// $ echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
//
// Method 2. If you want to set it as be done automatically, and don't want to do above everytime, make rules file in /etc/udev/rules.d/. For example,
// $ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules
// $ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
// $ sudo udevadm control --reload-rules
// $ sudo udevadm trigger --action=add
// $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
//
// or if you have another good idea that can be an alternatives,
// please give us advice via github issue https://github.com/ROBOTIS-GIT/DynamixelSDK/issues

#define GPIO_HALF_DUPLEX_DIRECTION 17

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

using namespace dynamixel;

PortHandlerLinux::PortHandlerLinux(const char *port_name) : packet_start_time_ms_(0.0), packet_timeout_ms_(0.0)
{
    is_using_ = false;
    setPortName(port_name);
}

void PortHandlerLinux::gpioHigh()
{
#if !defined(NIRYO_NED2) && (defined(__arm__) || defined(__aarch64__))
    digitalWrite(GPIO_HALF_DUPLEX_DIRECTION, HIGH);
#endif
}

void PortHandlerLinux::gpioLow()
{
#if !defined(NIRYO_NED2) && (defined(__arm__) || defined(__aarch64__))
    digitalWrite(GPIO_HALF_DUPLEX_DIRECTION, LOW);
#endif
}

/**
 * @brief PortHandlerLinux::openPort
 * @return
 *
 * setup half-duplex direction GPIO
 * see schema http:// support.robotis.com/en/product/actuator/dynamixel_x/xl-series_main.htm
 */
bool PortHandlerLinux::openPort()
{
#if !defined(NIRYO_NED2) && (defined(__arm__) || defined(__aarch64__))
    int res = wiringPiSetupGpio();

    if (res != 0)
    {
        return false;
    }

    pinMode(GPIO_HALF_DUPLEX_DIRECTION, OUTPUT);
    timespec wait_time = {0, static_cast<long>(500000)};
    pselect(0, NULL, NULL, NULL, &wait_time, NULL);
    gpioLow();
#endif

    serial_.open();
    return serial_.isOpen();
}

void PortHandlerLinux::closePort() { serial_.close(); }

void PortHandlerLinux::clearPort() { serial_.flush(); }

void PortHandlerLinux::flushInput() { serial_.flushInput(); }

void PortHandlerLinux::setPortName(const char *port_name) { serial_.setPort(port_name); }

const char *PortHandlerLinux::getPortName() { return serial_.getPort().c_str(); }

bool PortHandlerLinux::setBaudRate(const int baudrate)
{
    serial_.setBaudrate(baudrate);
    return serial_.getBaudrate() == baudrate;
}

int PortHandlerLinux::getBaudRate() { return serial_.getBaudrate(); }

int PortHandlerLinux::getBytesAvailable() { return serial_.available(); }

int PortHandlerLinux::readPort(uint8_t *packet, int length) { return serial_.read(packet, length); }

int PortHandlerLinux::writePort(uint8_t *packet, int length)
{
    gpioHigh();
    size_t written = serial_.write(packet, length);
    serial_.waitByteTimes(written);
    gpioLow();
    return written;
}

void PortHandlerLinux::setPacketTimeout(uint16_t packet_length)
{
    packet_start_time_ms_ = getCurrentTimeMs();
    uint32_t byte_time_ms = serial_.getByteTimeNs() / 1000000;
    packet_timeout_ms_ = (byte_time_ms * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerLinux::setPacketTimeout(double msec)
{
    packet_start_time_ms_ = getCurrentTimeMs();
    packet_timeout_ms_ = msec;
}

bool PortHandlerLinux::isPacketTimeout()
{
    if (getTimeSinceStart() > packet_timeout_ms_)
    {
        packet_timeout_ms_ = 0;
        return true;
    }
    return false;
}

double PortHandlerLinux::getCurrentTimeMs()
{
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.001 * 0.001);
}

double PortHandlerLinux::getTimeSinceStart()
{
    double time = getCurrentTimeMs() - packet_start_time_ms_;

    if (time < 0.0)
        packet_start_time_ms_ = getCurrentTimeMs();

    return time;
}

#endif
