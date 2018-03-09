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
# *********     Read and Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
# To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below variables yourself.
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

import os
import time

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

os.sys.path.append('~/workspace/DynamixelSDK/python/dynamixel_functions_py/')

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

# Control table address
#ADDR_PRO_TORQUE_ENABLE       = 6                          # Control table address is different in Dynamixel model
#ADDR_PRO_GOAL_POSITION       = 596
#ADDR_PRO_PRESENT_POSITION    = 611

ADDR_X_OPERATING_MODE       = 11
X_OPERATING_MODE_VELOCITY   = 1
X_OPERATING_MODE_POSITION   = 3
X_OPERATING_MODE_PWM        = 16
ADDR_X_PWM_LIMIT            = 36
ADDR_X_TORQUE_ENABLE        = 64
ADDR_X_GOAL_PWM             = 100
ADDR_X_GOAL_POSITION        = 116
ADDR_X_PRESENT_POSITION     = 132

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Default setting
DXL_IDS                     = [1, 2, 13, 14]                             # Dynamixel ID: 1
index                       = {DXL_IDS[0]: 0, DXL_IDS[1]: 1, DXL_IDS[2]: 0, DXL_IDS[3]: 1}
BAUDRATE                    = 1000000 #57600
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 2048 - 400                    # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2048 + 400                    # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold

PWM_LIMIT                   = 200

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = dynamixel.portHandler(DEVICENAME)

# Initialize PacketHandler Structs
dynamixel.packetHandler()

dxl_comm_result = COMM_TX_FAIL                              # Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

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

def init_actuator(dxl_id, operating_mode):
    # Enable Dynamixel Torque
    print "enabling pwm control"
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_X_OPERATING_MODE, operating_mode)
    print "limiting torque"
    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_X_PWM_LIMIT, PWM_LIMIT)
    print "enabling torque"
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_X_TORQUE_ENABLE, TORQUE_ENABLE)


def pwm_control(dxl_id, goal_pos):
    # Read present position
    dxl_present_position = dynamixel.read4ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_X_PRESENT_POSITION)

    goalpwm = (goal_pos - dxl_present_position) * 10
    goalpwm = PWM_LIMIT if goalpwm > PWM_LIMIT else -1 * PWM_LIMIT if goalpwm < -1 * PWM_LIMIT else goalpwm
    #goalpwm = -1 * goalpwm + 2048 if goalpwm < 0 else goalpwm
    #print "goalpwm", goalpwm
    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_X_GOAL_PWM, goalpwm)

    #print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (dxl_id, dxl_goal_position[index[dxl_id]], dxl_present_position))

    if not (abs(goal_pos - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_X_GOAL_PWM, 0)
        return True
    return False

def check_errors():
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)

    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
        return False
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
        return False
    return True

def disable_torque(dxl_id):
    # Disable Dynamixel Torque
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_X_TORQUE_ENABLE, TORQUE_DISABLE)

for dxl_id in DXL_IDS:
    init_actuator(dxl_id, X_OPERATING_MODE_PWM)

if check_errors():
    print("Dynamixel has been successfully connected")

steps_before_key = 100

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(ESC_ASCII_VALUE):
        break

    # Write goal position
    check_errors()

    start_time = time.time()
    for i in range(steps_before_key):
        for dxl_id in DXL_IDS:
            goal_pos = index[dxl_id]
            if pwm_control(dxl_id, dxl_goal_position[goal_pos]):
                index[dxl_id] = 0 if index[dxl_id] == 1 else 1
        check_errors()
    duration = time.time() - start_time
    print "duration: %.2fs" % (duration)
    print "%.2fHz" % (steps_before_key / duration)

    for dxl_id in DXL_IDS:
        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_X_GOAL_PWM, 0)
    # for dxl_id in DXL_IDS:
    #
    # print index


for dxl_id in DXL_IDS:
    disable_torque(dxl_id)

check_errors()

# Close port
dynamixel.closePort(port_num)
