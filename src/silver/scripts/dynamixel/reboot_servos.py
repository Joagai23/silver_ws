#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Connection settings
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DXL_IDS = list(range(18))  # IDs 0..17 (6 legs x 3 joints)

# Dynamixel Protocol 2.0 (X-Series) Control Table Addresses
ADDR_PRESENT_CURRENT = 126        # 2 bytes, signed int16, 2.69 mA / LSB
ADDR_PRESENT_INPUT_VOLTAGE = 144  # 2 bytes, uint16, 0.1 V / LSB

CURRENT_UNIT_MA = 2.69            # mA per LSB (XM430 / XH430 / XM540)
VOLTAGE_UNIT_V = 0.1              # V per LSB

# Expected 12V rail bounds for quick health flagging
MIN_OK_VOLTAGE = 11.0
MAX_OK_VOLTAGE = 13.5


def to_signed_16bit(val: int) -> int:
    return val - 65536 if val > 32767 else val


def read_servo_telemetry(port_handler, packet_handler, dxl_id: int):
    """Read Present Input Voltage (V) and Present Current (mA) for a single servo."""
    raw_volt, comm_v, err_v = packet_handler.read2ByteTxRx(
        port_handler, dxl_id, ADDR_PRESENT_INPUT_VOLTAGE
    )
    raw_curr, comm_c, err_c = packet_handler.read2ByteTxRx(
        port_handler, dxl_id, ADDR_PRESENT_CURRENT
    )

    if comm_v != COMM_SUCCESS or comm_c != COMM_SUCCESS:
        err_msg = packet_handler.getTxRxResult(comm_v if comm_v != COMM_SUCCESS else comm_c)
        return None, None, f"COMM ERROR: {err_msg}"

    hw_err = err_v or err_c
    voltage_v = raw_volt * VOLTAGE_UNIT_V
    current_ma = to_signed_16bit(raw_curr) * CURRENT_UNIT_MA

    if hw_err != 0:
        return voltage_v, current_ma, f"HW ALERT: {packet_handler.getRxPacketError(hw_err)}"

    if not (MIN_OK_VOLTAGE <= voltage_v <= MAX_OK_VOLTAGE):
        return voltage_v, current_ma, "WARN: Voltage out of 12V range"

    return voltage_v, current_ma, "OK"


def print_telemetry_table(port_handler, packet_handler, dry_run: bool = False):
    print("\n--- Servo Voltage & Current Diagnostics ---")
    print(f"{'Leg':<6} {'ID':<6} {'Voltage (V)':<14} {'Current (mA)':<15} {'Status'}")
    print("-" * 62)

    online_count = 0
    for dxl_id in DXL_IDS:
        leg_idx = dxl_id // 3
        if dry_run:
            v_str, c_str, status = "12.0 V", "35.0 mA", "OK (dry-run)"
            online_count += 1
        else:
            volt, curr, status = read_servo_telemetry(port_handler, packet_handler, dxl_id)
            if volt is None:
                v_str, c_str = "---", "---"
            else:
                v_str = f"{volt:5.1f} V"
                c_str = f"{curr:7.1f} mA"
                online_count += 1

        print(f"Leg {leg_idx:<2} {dxl_id:03d}    {v_str:<14} {c_str:<15} {status}")

    print("-" * 62)
    print(f"Responding servos: {online_count}/{len(DXL_IDS)}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Reboot SILVER2 Dynamixel servos and read voltage/current telemetry."
    )
    parser.add_argument(
        '--dry-run', action='store_true', help='Simulate without opening /dev/ttyUSB0'
    )
    parser.add_argument(
        '--check-only', action='store_true', help='Skip reboot and only read voltage & current'
    )
    args = parser.parse_args()

    port_handler = None
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not args.dry_run:
        port_handler = PortHandler(DEVICENAME)
        if not port_handler.openPort():
            print(f"[ERROR] Failed to open port {DEVICENAME}")
            sys.exit(1)
        if not port_handler.setBaudRate(BAUDRATE):
            print(f"[ERROR] Failed to set baudrate to {BAUDRATE}")
            port_handler.closePort()
            sys.exit(1)
        print(f"[INFO] Connected to {DEVICENAME} @ {BAUDRATE} bps")
    else:
        print(f"[INFO] Dry-run mode enabled (not opening {DEVICENAME})")

    try:
        if not args.check_only:
            input(f"Press [Enter] to reboot {len(DXL_IDS)} servos (or Ctrl+C to abort)...")
            print("\nBroadcasting reboot commands (actuator LEDs will flicker during reset)...")

            succeeded = []
            failed = []

            for dxl_id in DXL_IDS:
                if args.dry_run:
                    print(f"[ID:{dxl_id:03d}] Reboot signal acknowledged (dry-run).")
                    succeeded.append(dxl_id)
                    continue

                comm_result, dxl_error = packet_handler.reboot(port_handler, dxl_id)
                if comm_result != COMM_SUCCESS:
                    print(f"[ID:{dxl_id:03d}] Reboot FAILED: {packet_handler.getTxRxResult(comm_result)}")
                    failed.append(dxl_id)
                elif dxl_error != 0:
                    print(f"[ID:{dxl_id:03d}] Reboot HW error: {packet_handler.getRxPacketError(dxl_error)}")
                    failed.append(dxl_id)
                else:
                    print(f"[ID:{dxl_id:03d}] Reboot signal acknowledged successfully.")
                    succeeded.append(dxl_id)

            print("\n--- Reboot Summary ---")
            print(f"Succeeded ({len(succeeded)}/{len(DXL_IDS)}): {succeeded}")
            if failed:
                print(f"Failed ({len(failed)}/{len(DXL_IDS)}): {failed}")

            if not args.dry_run:
                print("\nWaiting 1.0s for servos to finish booting before reading telemetry...")
                time.sleep(1.0)

        print_telemetry_table(port_handler, packet_handler, dry_run=args.dry_run)

    except KeyboardInterrupt:
        print("\n[INFO] Aborted by user.")
    finally:
        if port_handler is not None:
            port_handler.closePort()
            print("[INFO] Serial port closed cleanly.")


if __name__ == '__main__':
    main()