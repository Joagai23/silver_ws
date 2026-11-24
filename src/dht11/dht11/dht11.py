#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import board
import adafruit_dht
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, String

# Physical canister wiring map (18 leg joints + Control + Camera canisters)
SENSOR_PIN_MAP = [
    ('C0', 25, board.D25), ('F0', 24, board.D24), ('T0', 23, board.D23),
    ('C1', 26, board.D26), ('F1',  6, board.D6),  ('T1',  5, board.D5),
    ('C2', 22, board.D22), ('F2', 27, board.D27), ('T2', 17, board.D17),
    ('C3', 20, board.D20), ('F3',  7, board.D7),  ('T3', 15, board.D15),
    ('C4', 14, board.D14), ('F4',  4, board.D4),  ('T4', 16, board.D16),
    ('C5',  9, board.D9),  ('F5', 10, board.D10), ('T5', 21, board.D21),
    ('ControlC', 11, board.D11),
    ('CameraC',  13, board.D13),
]

# ANSI terminal colors
RED_BG = "\033[1;97;41m"   # Bold white text on red background
YELLOW = "\033[1;33m"      # Bold yellow text
RESET = "\033[0m"

class DHT11MultiSensorNode(Node):

    def __init__(self):
        super().__init__('dht11')

        # Configurable parameters
        self.declare_parameter('read_interval', 2.0)       # Seconds between full scans
        self.declare_parameter('rh_rise_threshold', 10.0)  # % RH rise above baseline (X + 10)
        self.declare_parameter('temp_threshold', 50.0)     # Celsius overheat warning

        self.read_interval = float(self.get_parameter('read_interval').value)
        self.rh_rise_threshold = float(self.get_parameter('rh_rise_threshold').value)
        self.temp_threshold = float(self.get_parameter('temp_threshold').value)

        self.sensors = []
        self.initial_humidities = {}

        for loc, gpio_num, board_pin in SENSOR_PIN_MAP:
            try:
                device = adafruit_dht.DHT11(board_pin, use_pulseio=False)
                self.sensors.append((loc, gpio_num, device))
            except Exception as e:
                self.get_logger().error(f"Failed to init DHT11 at {loc} (GPIO {gpio_num}): {e}")
                self.sensors.append((loc, gpio_num, None))

        self.temperature_publisher_ = self.create_publisher(
            Float64MultiArray, '/dht11/temperature', 10
        )
        self.humidity_publisher_ = self.create_publisher(
            Float64MultiArray, '/dht11/humidity', 10
        )

        # Latched topic so rosbags and subscribers always know the exact canister order
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.locations_publisher_ = self.create_publisher(
            String, '/dht11/locations', latched_qos
        )
        locations_csv = ",".join(loc for loc, _, _ in SENSOR_PIN_MAP)
        self.locations_publisher_.publish(String(data=locations_csv))

        self.timer = self.create_timer(self.read_interval, self.timer_callback)
        self.get_logger().info(
            f"DHT11 node initialized for {len(self.sensors)} canisters every {self.read_interval}s "
            f"(Alerts: RH >= RH0 + {self.rh_rise_threshold:.0f}%, Temp >= {self.temp_threshold:.0f}C)."
        )

    def timer_callback(self):
        temperatures = []
        humidities = []
        leak_warnings = []
        temp_warnings = []

        for loc, gpio_num, device in self.sensors:
            if device is None:
                temperatures.append(float('nan'))
                humidities.append(float('nan'))
                continue

            try:
                t = device.temperature
                h = device.humidity
                t_val = float(t) if t is not None else float('nan')
                h_val = float(h) if h is not None else float('nan')
                temperatures.append(t_val)
                humidities.append(h_val)

                # Record baseline X on first valid read, then warn if RH >= X + threshold
                if h is not None:
                    if loc not in self.initial_humidities:
                        self.initial_humidities[loc] = h_val
                        self.get_logger().info(
                            f"Canister '{loc}' (GPIO {gpio_num}) baseline recorded: "
                            f"RH0 = {h_val:.1f}% | T0 = {t_val:.1f} C"
                        )
                    else:
                        rh0 = self.initial_humidities[loc]
                        if (h_val - rh0) >= self.rh_rise_threshold:
                            leak_warnings.append(
                                f"{loc} (GPIO {gpio_num}): {h_val:.1f}% (+{h_val - rh0:.1f}% above {rh0:.1f}%)"
                            )

                if t is not None and t_val >= self.temp_threshold:
                    temp_warnings.append(
                        f"{loc} (GPIO {gpio_num}): {t_val:.1f} C (>= {self.temp_threshold:.0f} C)"
                    )

            except RuntimeError:
                temperatures.append(float('nan'))
                humidities.append(float('nan'))
            except Exception as e:
                self.get_logger().warn(
                    f"DHT11 '{loc}' (GPIO {gpio_num}) error: {e}", throttle_duration_sec=10.0
                )
                temperatures.append(float('nan'))
                humidities.append(float('nan'))

            time.sleep(0.01)

        if leak_warnings:
            self.get_logger().warn(
                f"{RED_BG} CANISTER HUMIDITY RISE (+{self.rh_rise_threshold:.0f}%): "
                f"{' | '.join(leak_warnings)} {RESET}",
                throttle_duration_sec=5.0,
            )
        if temp_warnings:
            self.get_logger().warn(
                f"{YELLOW}CANISTER HIGH TEMP: {' | '.join(temp_warnings)}{RESET}",
                throttle_duration_sec=5.0,
            )

        self.temperature_publisher_.publish(Float64MultiArray(data=temperatures))
        self.humidity_publisher_.publish(Float64MultiArray(data=humidities))

    def cleanup(self):
        for _, _, device in self.sensors:
            if device is not None:
                try:
                    device.exit()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = DHT11MultiSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()