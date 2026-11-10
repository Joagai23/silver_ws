#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
reboot_servos.py

Safely issues Protocol 2.0 reboot instructions across all target Dynamixel
actuators on the serial bus with verified status reporting.
"""

import sys
import time
from typing import List
from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

# --- Bus Configuration ---
DEVICENAME: str = "/dev/ttyUSB0"
BAUDRATE: int = 1000000
PROTOCOL_VERSION: float = 2.0

# Target IDs: 18 actuators (IDs 0 through 17)
TARGET_IDS: List[int] = list(range(0, 18))

# --- Initialization ---
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print(f"[ERROR] Failed to open port: {DEVICENAME}")
    sys.exit(1)

if not portHandler.setBaudRate(BAUDRATE):
    print(f"[ERROR] Failed to set baudrate to {BAUDRATE} bps")
    portHandler.closePort()
    sys.exit(1)

print(f"[INFO] Connected to {DEVICENAME} @ {BAUDRATE} bps")

try:
    user_prompt = input(f"Press [Enter] to reboot {len(TARGET_IDS)} servos (or Ctrl+C to abort)... ")

    print("\nBroadcasting reboot commands (actuator LEDs will flicker during reset)...")
    succeeded_ids = []
    failed_ids = []

    for dxl_id in TARGET_IDS:
        comm_result, error = packetHandler.reboot(portHandler, dxl_id)

        if comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id:03d}] Communication failure: {packetHandler.getTxRxResult(comm_result)}")
            failed_ids.append(dxl_id)
        elif error != 0:
            print(f"[ID:{dxl_id:03d}] Hardware error: {packetHandler.getRxPacketError(error)}")
            failed_ids.append(dxl_id)
        else:
            print(f"[ID:{dxl_id:03d}] Reboot signal acknowledged successfully.")
            succeeded_ids.append(dxl_id)

        # Allow time for the transceiver line to settle before issuing the next packet
        time.sleep(0.05)

    print("\n--- Reboot Summary ---")
    print(f"Succeeded ({len(succeeded_ids)}/{len(TARGET_IDS)}): {succeeded_ids}")
    if failed_ids:
        print(f"Failed ({len(failed_ids)}/{len(TARGET_IDS)}): {failed_ids}")

finally:
    portHandler.closePort()
    print("[INFO] Serial port closed cleanly.")