#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon), Andreas Gerken

#
# *********     Read and Write Example for PWM control   *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is designed for using a Dynamixel X series actuator, and an USB2DYNAMIXEL.
# Be sure that Dynamixel X properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

os.sys.path.append('../dynamixel_functions_py')             # Path setting

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

# Control table address

ADDR_X_OPERATING_MODE       = 11
X_OPERATING_MODE_VELOCITY   = 1
X_OPERATING_MODE_POSITION   = 3
X_OPERATING_MODE_PWM        = 16
ADDR_X_PWM_LIMIT            = 36
X_PWM_LIMIT                 = 885                           # Initial value from datasheet
ADDR_X_GOAL_PWM             = 100

ADDR_X_TORQUE_ENABLE        = 64
ADDR_X_GOAL_POSITION        = 116
ADDR_X_PRESENT_POSITION     = 132

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 3                             # Dynamixel ID: 1
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque


ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = dynamixel.portHandler(DEVICENAME)

# Initialize PacketHandler Structs
dynamixel.packetHandler()

index = 0
dxl_goal_pwm = [-200,200]
dxl_comm_result = COMM_TX_FAIL                              # Communication result

dxl_error = 0                                               # Dynamixel error
dxl_present_position = 0                                    # Present position

# Open port
if dynamixel.openPort(port_num):
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if dynamixel.setBaudRate(port_num, BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    print("Press any key to terminate...")
    getch()
    quit()

print("Setting operating mode to pwm")
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_X_OPERATING_MODE, X_OPERATING_MODE_PWM)

print("Setting pwm limit")
dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_X_PWM_LIMIT, X_PWM_LIMIT)

# Enable Dynamixel Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_X_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
if dxl_comm_result != COMM_SUCCESS:
    print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
elif dxl_error != 0:
    print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
else:
    print("Dynamixel has been successfully connected")


while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(ESC_ASCII_VALUE):
        break

    # Write goal pwm
    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_X_GOAL_PWM, dxl_goal_pwm[index])
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))

    # Read present position
    dxl_present_position = dynamixel.read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_X_PRESENT_POSITION)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))

    print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_pwm[index], dxl_present_position))

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Disable Dynamixel Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_X_TORQUE_ENABLE, TORQUE_DISABLE)
dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
if dxl_comm_result != COMM_SUCCESS:
    print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
elif dxl_error != 0:
    print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))


# Close port
dynamixel.closePort(port_num)
