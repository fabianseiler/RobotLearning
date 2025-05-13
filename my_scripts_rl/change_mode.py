#!/usr/bin/env python3

import os
import sys
import time
import argparse

# Try to import dynamixel_sdk after ensuring the serial package is installed
try:
    from dynamixel_sdk import *
except ModuleNotFoundError:
    print("Error: PySerial package is missing. Install it with:")
    print("    pip3 install pyserial")
    print("or")
    print("    sudo apt-get install python3-serial")
    sys.exit(1)

# Control table addresses for Dynamixel motors
ADDR_OPERATING_MODE = 11       # Address for Operating Mode
ADDR_TORQUE_ENABLE = 64        # Address for Torque Enable

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'

def change_operating_mode(mode_value):
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    try:
        if not portHandler.openPort():
            print("Failed to open the port.")
            return False

        if not portHandler.setBaudRate(BAUDRATE):
            print("Failed to set baudrate.")
            portHandler.closePort()
            return False

        for dxl_id in range(11, 18):
            print(f"Processing motor ID {dxl_id}...")

            # Disable torque
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
                portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)

            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error disabling torque on ID {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
                continue
            elif dxl_error != 0:
                print(f"Packet error disabling torque on ID {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")
                continue

            # Determine the target mode based on the input mode and motor ID
            target_mode = mode_value
            if mode_value == 3 and dxl_id == 17:
                target_mode = 5  # Change motor 17 to current-based position control mode
                print(f"Motor ID {dxl_id} is being set to special mode {target_mode} instead of {mode_value}")

            # Set new operating mode
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
                portHandler, dxl_id, ADDR_OPERATING_MODE, target_mode)

            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error setting mode for ID {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
                continue
            elif dxl_error != 0:
                print(f"Packet error setting mode for ID {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")
                continue

            print(f"Motor ID {dxl_id} set to mode {target_mode}")
            time.sleep(0.1)

        portHandler.closePort()
        print("All motors processed.")
        return True

    except Exception as e:
        print(f"An exception occurred: {e}")
        if portHandler.is_open:
            portHandler.closePort()
        return False


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Change operating mode of Dynamixel motors.")
    parser.add_argument('--mode', type=int, required=True, help="Operating mode to set (e.g. 3 = position, 0 = current, etc.)")
    args = parser.parse_args()

    print(f"Changing Dynamixel motors (ID 11-17) to operating mode {args.mode}...")
    result = change_operating_mode(args.mode)

    if result:
        print("Successfully changed operating mode for all motors.")
    else:
        print("Failed to change operating mode for some or all motors.")
