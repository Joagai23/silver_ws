#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import Float64, Float64MultiArray, String
from sensor_msgs.msg import Imu, JointState, Temperature

CANISTER_LOCATIONS = [
    'C0', 'F0', 'T0',
    'C1', 'F1', 'T1',
    'C2', 'F2', 'T2',
    'C3', 'F3', 'T3',
    'C4', 'F4', 'T4',
    'C5', 'F5', 'T5',
    'ControlC',
    'CameraC',
]

def quat_to_euler_deg(x: float, y: float, z: float, w: float):
    """Convert quaternion (x, y, z, w) to Roll, Pitch, Yaw in degrees."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class SilverSensorLogger(Node):

    def __init__(self):
        super().__init__('silver_sensor_logger')

        self.declare_parameter('log_rate', 0.5)    # Console refresh rate in Hz (default: every 2.0s)
        self.declare_parameter('in_place', True)   # Overwrite the 4 dashboard lines in place

        log_rate = float(self.get_parameter('log_rate').value)
        self.in_place = bool(self.get_parameter('in_place').value)
        self._lines_drawn = 0

        self.data = {
            'vin_V': float('nan'),
            'current_A': float('nan'),
            'power_W': float('nan'),
            'canister_P_mbar': float('nan'),
            'canister_T_C': float('nan'),
            'canister_RH_pct': float('nan'),
            'ext_P_mbar': float('nan'),
            'ext_T_C': float('nan'),
            'ext_depth_m': float('nan'),
            'imu_roll_deg': float('nan'),
            'imu_pitch_deg': float('nan'),
            'imu_yaw_deg': float('nan'),
            'imu_T_C': float('nan'),
            'imu_calib': '',
        }

        for loc in CANISTER_LOCATIONS:
            self.data[f'dht_{loc}_T_C'] = float('nan')
            self.data[f'dht_{loc}_RH_pct'] = float('nan')

        self.num_joints = 18
        self.joint_names_logged = False
        for i in range(self.num_joints):
            self.data[f'eff_{i:02d}'] = float('nan')

        # --- Subscriptions ---
        self.create_subscription(Float64, '/ltc2945/vin', lambda m: self._set('vin_V', m.data), 10)
        self.create_subscription(Float64, '/ltc2945/current', lambda m: self._set('current_A', m.data), 10)
        self.create_subscription(Float64, '/ltc2945/power', lambda m: self._set('power_W', m.data), 10)

        self.create_subscription(Float64, '/ms8607_i2c0/pressure', lambda m: self._set('canister_P_mbar', m.data), 10)
        self.create_subscription(Float64, '/ms8607_i2c0/temperature', lambda m: self._set('canister_T_C', m.data), 10)
        self.create_subscription(Float64, '/ms8607_i2c0/humidity', lambda m: self._set('canister_RH_pct', m.data), 10)

        self.create_subscription(Float64, '/ms5837/pressure', lambda m: self._set('ext_P_mbar', m.data), 10)
        self.create_subscription(Float64, '/ms5837/temperature', lambda m: self._set('ext_T_C', m.data), 10)
        self.create_subscription(Float64, '/ms5837/depth', lambda m: self._set('ext_depth_m', m.data), 10)

        self.create_subscription(Imu, '/bno055/imu', self._imu_cb, 10)
        self.create_subscription(Temperature, '/bno055/temp', lambda m: self._set('imu_T_C', m.temperature), 10)
        self.create_subscription(String, '/bno055/calib_status', lambda m: self._set('imu_calib', m.data), 10)

        self.create_subscription(Float64MultiArray, '/dht11/temperature', self._dht_temp_cb, 10)
        self.create_subscription(Float64MultiArray, '/dht11/humidity', self._dht_hum_cb, 10)

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # Reset cursor anchor if any node emits WARN/ERROR/FATAL so alerts aren't overwritten
        self.create_subscription(Log, '/rosout', self._rosout_cb, 10)

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / log_rate, self._print_status)

        self.get_logger().info(
            f"SILVER2 console sensor monitor started ({log_rate} Hz, in_place={self.in_place})."
        )

    def _rosout_cb(self, msg: Log):
        # Level 30 = WARN, 40 = ERROR, 50 = FATAL
        if msg.level >= 30 and msg.name != self.get_name():
            self._lines_drawn = 0

    def _set(self, key: str, val):
        self.data[key] = val

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        r, p, y = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        self.data['imu_roll_deg'] = round(r, 2)
        self.data['imu_pitch_deg'] = round(p, 2)
        self.data['imu_yaw_deg'] = round(y, 2)

    def _dht_temp_cb(self, msg: Float64MultiArray):
        for loc, val in zip(CANISTER_LOCATIONS, msg.data):
            if not math.isnan(val):
                self.data[f'dht_{loc}_T_C'] = val

    def _dht_hum_cb(self, msg: Float64MultiArray):
        for loc, val in zip(CANISTER_LOCATIONS, msg.data):
            if not math.isnan(val):
                self.data[f'dht_{loc}_RH_pct'] = val

    def _joint_state_cb(self, msg: JointState):
        if not self.joint_names_logged and msg.name:
            mapping = ", ".join(f"{i:02d}:{name}" for i, name in enumerate(msg.name))
            self.get_logger().info(f"Joint effort mapping -> {mapping}")
            self._lines_drawn = 0
            self.joint_names_logged = True

        for i, val in enumerate(msg.effort[:self.num_joints]):
            self.data[f'eff_{i:02d}'] = float(val)

    def _fmt_rh(self, loc: str) -> str:
        rh = self.data[f'dht_{loc}_RH_pct']
        return f"{rh:.0f}%" if not math.isnan(rh) else "--"

    def _fmt_temp(self, loc: str) -> str:
        t = self.data[f'dht_{loc}_T_C']
        return f"{t:.1f}C" if not math.isnan(t) else "--"

    def _print_status(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        rh_parts = []
        temp_parts = []
        for leg in range(6):
            rh_vals = [self._fmt_rh(f"{j}{leg}") for j in ('C', 'F', 'T')]
            t_vals = [self._fmt_temp(f"{j}{leg}") for j in ('C', 'F', 'T')]
            rh_parts.append(f"L{leg}:{'/'.join(rh_vals)}")
            temp_parts.append(f"L{leg}:{'/'.join(t_vals)}")

        rh_parts.extend([f"Ctrl:{self._fmt_rh('ControlC')}", f"Cam:{self._fmt_rh('CameraC')}"])
        temp_parts.extend([f"Ctrl:{self._fmt_temp('ControlC')}", f"Cam:{self._fmt_temp('CameraC')}"])

        eff_parts = []
        for leg in range(6):
            vals = [self.data[f'eff_{leg * 3 + j:02d}'] for j in range(3)]
            if all(math.isnan(v) for v in vals):
                eff_parts.append(f"L{leg}:--")
            else:
                v_str = "/".join(f"{v:.2f}" if not math.isnan(v) else "--" for v in vals)
                eff_parts.append(f"L{leg}:{v_str}")

        lines = [
            (
                f"[{elapsed:6.1f}s] Bat: {self.data['vin_V']:.2f}V, {self.data['current_A']:.2f}A, {self.data['power_W']:.1f}W | "
                f"BatCan: {self.data['canister_RH_pct']:.1f}% RH, {self.data['canister_T_C']:.1f}C, {self.data['canister_P_mbar']:.0f}mbar | "
                f"Depth: {self.data['ext_depth_m']:.2f}m ({self.data['ext_T_C']:.1f}C) | "
                f"RPY: ({self.data['imu_roll_deg']:.1f}, {self.data['imu_pitch_deg']:.1f}, {self.data['imu_yaw_deg']:.1f}) deg"
            ),
            f"[{elapsed:6.1f}s] Canisters RH: " + " | ".join(rh_parts),
            f"[{elapsed:6.1f}s] Canisters T : " + " | ".join(temp_parts),
            f"[{elapsed:6.1f}s] Effort      : " + " | ".join(eff_parts),
        ]

        if not self.in_place:
            for line in lines:
                self.get_logger().info(line)
            return

        if self._lines_drawn > 0:
            sys.stdout.write(f"\033[{self._lines_drawn}A")

        for line in lines:
            sys.stdout.write(f"\033[2K\r{line}\n")

        sys.stdout.flush()
        self._lines_drawn = len(lines)

def main(args=None):
    rclpy.init(args=args)
    node = SilverSensorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.in_place and node._lines_drawn > 0:
            sys.stdout.write("\n")
            sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()