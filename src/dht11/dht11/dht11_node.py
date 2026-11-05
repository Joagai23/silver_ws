#!/usr/bin/env python3
import time
import board
import adafruit_dht
import rclpy
from rclpy.node import Node
from silver_interfaces.msg import Dhtarray


class DHT11MultiSensorNode(Node):
    def __init__(self):
        super().__init__('dht11_node')

        # Declare configurable parameters
        self.declare_parameter('read_interval', 5.0)
        self.declare_parameter('temp_threshold', 60.0)      # Celsius
        self.declare_parameter('humidity_threshold', 80.0)  # Percentage (%)

        self.read_interval = self.get_parameter('read_interval').value
        self.temp_threshold = self.get_parameter('temp_threshold').value
        self.humidity_threshold = self.get_parameter('humidity_threshold').value

        # Pin mapping matching physical canister wiring
        self.sensor_pin_map = {
            'C0': board.D25, 'F0': board.D24, 'T0': board.D23,
            'C1': board.D26, 'F1': board.D6,  'T1': board.D5,
            'C2': board.D22, 'F2': board.D27, 'T2': board.D17,
            'C3': board.D20, 'F3': board.D7,  'T3': board.D15,
            'C4': board.D14, 'F4': board.D4,  'T4': board.D16,
            'C5': board.D9,  'F5': board.D10, 'T5': board.D21,
            'ControlC': board.D11,
            'CameraC': board.D13,
        }

        self.locations_sorted = list(self.sensor_pin_map.keys())
        self.devices = {}

        for loc, pin in self.sensor_pin_map.items():
            try:
                self.devices[loc] = adafruit_dht.DHT11(pin, use_pulseio=False)
            except Exception as e:
                self.get_logger().error(f"Failed to initialize DHT11 at {loc} (pin {pin}): {e}")
                self.devices[loc] = None

        self.publisher_ = self.create_publisher(Dhtarray, '/dht11', 10)
        self.timer = self.create_timer(self.read_interval, self.read_all_sensors)

        self.get_logger().info(
            f"DHT11 Node started. Polling {len(self.locations_sorted)} sensors every {self.read_interval}s "
            f"(Alerts: Temp > {self.temp_threshold}C, Hum > {self.humidity_threshold}%)."
        )

    def read_all_sensors(self):
        msg = Dhtarray()
        msg.locations = self.locations_sorted
        temperatures = []
        humidities = []

        for loc in self.locations_sorted:
            device = self.devices.get(loc)
            if device is None:
                temperatures.append(-1.0)
                humidities.append(-1.0)
                continue

            try:
                temp = device.temperature
                hum = device.humidity
                if temp is not None and hum is not None:
                    temp_f = float(temp)
                    hum_f = float(hum)
                    temperatures.append(temp_f)
                    humidities.append(hum_f)

                    # Hardware safety threshold checks
                    if hum_f >= self.humidity_threshold:
                        self.get_logger().error(
                            f"[LEAK/HUMIDITY WARNING] Canister '{loc}' humidity is {hum_f:.1f}% "
                            f"(Threshold: {self.humidity_threshold:.1f}%)!"
                        )
                    if temp_f >= self.temp_threshold:
                        self.get_logger().error(
                            f"[OVERHEAT WARNING] Canister '{loc}' temperature is {temp_f:.1f}C "
                            f"(Threshold: {self.temp_threshold:.1f}C)!"
                        )
                else:
                    temperatures.append(-1.0)
                    humidities.append(-1.0)
            except RuntimeError:
                temperatures.append(-1.0)
                humidities.append(-1.0)
            except Exception as e:
                self.get_logger().warn(f"Unexpected error on sensor {loc}: {e}")
                temperatures.append(-1.0)
                humidities.append(-1.0)

            time.sleep(0.01)

        msg.temperatures = temperatures
        msg.humidities = humidities
        self.publisher_.publish(msg)

    def destroy_node(self):
        for loc, device in self.devices.items():
            if device is not None:
                try:
                    device.exit()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DHT11MultiSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()