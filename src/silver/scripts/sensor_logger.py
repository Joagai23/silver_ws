#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String
from sensor_msgs.msg import Imu, JointState, Temperature

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

        default_log_dir = os.path.expanduser('~/silver_logs')
        self.declare_parameter('log_dir', default_log_dir)
        self.declare_parameter('log_rate', 2.0)  # Hz

        log_dir = self.get_parameter('log_dir').value
        log_rate = float(self.get_parameter('log_rate').value)
        os.makedirs(log_dir, exist_ok=True)

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'silver2_sensors_{timestamp_str}.csv')

        # Latest state buffer (initialized to NaN until first message arrives)
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

        self.dht_pins = [5, 6, 17, 22, 23, 26, 27]
        for pin in self.dht_pins:
            self.data[f'dht_g{pin}_T_C'] = float('nan')
            self.data[f'dht_g{pin}_RH_pct'] = float('nan')

        self.num_joints = 18
        self.joint_names_logged = False
        for i in range(self.num_joints):
            self.data[f'eff_{i:02d}'] = float('nan')

        # --- Subscriptions ---
        # LTC2945 Battery Monitor (i2c-0)
        self.create_subscription(Float64, '/ltc2945/vin', lambda m: self._set('vin_V', m.data), 10)
        self.create_subscription(Float64, '/ltc2945/current', lambda m: self._set('current_A', m.data), 10)
        self.create_subscription(Float64, '/ltc2945/power', lambda m: self._set('power_W', m.data), 10)

        # MS8607 Battery Canister Internal P/T/RH (i2c-0)
        self.create_subscription(Float64, '/ms8607_i2c0/pressure', lambda m: self._set('canister_P_mbar', m.data), 10)
        self.create_subscription(Float64, '/ms8607_i2c0/temperature', lambda m: self._set('canister_T_C', m.data), 10)
        self.create_subscription(Float64, '/ms8607_i2c0/humidity', lambda m: self._set('canister_RH_pct', m.data), 10)

        # MS5837 External Bar30 Depth Sensor (i2c-1)
        self.create_subscription(Float64, '/ms5837/pressure', lambda m: self._set('ext_P_mbar', m.data), 10)
        self.create_subscription(Float64, '/ms5837/temperature', lambda m: self._set('ext_T_C', m.data), 10)
        self.create_subscription(Float64, '/ms5837/depth', lambda m: self._set('ext_depth_m', m.data), 10)

        # BNO055 IMU (i2c-1)
        self.create_subscription(Imu, '/bno055/imu', self._imu_cb, 10)
        self.create_subscription(Temperature, '/bno055/temp', lambda m: self._set('imu_T_C', m.temperature), 10)
        self.create_subscription(String, '/bno055/calib_status', lambda m: self._set('imu_calib', m.data), 10)

        # DHT11 Canister Temp/Humidity (7 GPIO pins)
        self.create_subscription(Float64MultiArray, '/dht11/temperature', self._dht_temp_cb, 10)
        self.create_subscription(Float64MultiArray, '/dht11/humidity', self._dht_hum_cb, 10)

        # Joint States (per-servo effort / current from ros2_control)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # Open CSV and write header
        self.csv_file = open(self.csv_path, mode='w', newline='', encoding='utf-8')
        self.fieldnames = ['timestamp_iso', 'elapsed_s'] + list(self.data.keys())
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.csv_file.flush()

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / log_rate, self._log_row)

        self.get_logger().info(f"Logging SILVER2 sensors at {log_rate} Hz -> {self.csv_path}")

    def _set(self, key: str, val):
        self.data[key] = val

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        r, p, y = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        self.data['imu_roll_deg'] = round(r, 2)
        self.data['imu_pitch_deg'] = round(p, 2)
        self.data['imu_yaw_deg'] = round(y, 2)

    def _dht_temp_cb(self, msg: Float64MultiArray):
        for pin, val in zip(self.dht_pins, msg.data):
            if not math.isnan(val):
                self.data[f'dht_g{pin}_T_C'] = val

    def _dht_hum_cb(self, msg: Float64MultiArray):
        for pin, val in zip(self.dht_pins, msg.data):
            if not math.isnan(val):
                self.data[f'dht_g{pin}_RH_pct'] = val

    def _joint_state_cb(self, msg: JointState):
        if not self.joint_names_logged and msg.name:
            mapping = ", ".join(f"{i:02d}:{name}" for i, name in enumerate(msg.name))
            self.get_logger().info(f"Joint effort mapping -> {mapping}")
            self.joint_names_logged = True

        for i, val in enumerate(msg.effort[:self.num_joints]):
            self.data[f'eff_{i:02d}'] = float(val)

    def _log_row(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        row = {
            'timestamp_iso': datetime.now().isoformat(timespec='milliseconds'),
            'elapsed_s': f"{elapsed:.3f}",
        }
        for k, v in self.data.items():
            row[k] = f"{v:.4f}" if isinstance(v, float) and not math.isnan(v) else v

        self.writer.writerow(row)
        self.csv_file.flush()

        self.get_logger().info(
            f"[{elapsed:6.1f}s] Bat: {self.data['vin_V']:.2f}V, {self.data['current_A']:.2f}A, {self.data['power_W']:.1f}W | "
            f"Canister: {self.data['canister_RH_pct']:.1f}% RH, {self.data['canister_T_C']:.1f}C, {self.data['canister_P_mbar']:.0f}mbar | "
            f"Depth: {self.data['ext_depth_m']:.2f}m ({self.data['ext_T_C']:.1f}C) | "
            f"RPY: ({self.data['imu_roll_deg']:.1f}, {self.data['imu_pitch_deg']:.1f}, {self.data['imu_yaw_deg']:.1f}) deg",
            throttle_duration_sec=2.0,
        )

        dht_parts = []
        for pin in self.dht_pins:
            rh = self.data[f'dht_g{pin}_RH_pct']
            t = self.data[f'dht_g{pin}_T_C']
            rh_s = f"{rh:.0f}%" if not math.isnan(rh) else "--"
            t_s = f"{t:.1f}C" if not math.isnan(t) else "--"
            dht_parts.append(f"G{pin}:{rh_s}/{t_s}")

        self.get_logger().info(
            f"[{elapsed:6.1f}s] DHT11: " + " | ".join(dht_parts),
            throttle_duration_sec=2.0,
        )

        eff_parts = []
        for leg in range(6):
            vals = [self.data[f'eff_{leg * 3 + j:02d}'] for j in range(3)]
            if all(math.isnan(v) for v in vals):
                eff_parts.append(f"L{leg}:--")
            else:
                v_str = "/".join(f"{v:.2f}" if not math.isnan(v) else "--" for v in vals)
                eff_parts.append(f"L{leg}:{v_str}")

        self.get_logger().info(
            f"[{elapsed:6.1f}s] Effort: " + " | ".join(eff_parts),
            throttle_duration_sec=2.0,
        )

    def close_file(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            print(f"[INFO] [silver_sensor_logger]: Saved sensor log to {self.csv_path}")


def main(args=None):
    rclpy.init(args=args)
    node = SilverSensorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_file()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
