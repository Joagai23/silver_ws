import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import board
import adafruit_dht

# BCM GPIO pins matching the Main Board schematic
DHT_PINS = [
    (5, board.D5),
    (6, board.D6),
    (17, board.D17),
    (22, board.D22),
    (23, board.D23),
    (26, board.D26),
    (27, board.D27),
]

# Safety thresholds for canister monitoring
DHT_RH_RISE_WARN = 10.0  # % RH rise above startup baseline (X + 10)
DHT_TEMP_WARN = 50.0     # °C internal canister overtemperature warning

# ANSI terminal colors
RED_BG = "\033[1;97;41m"   # Bold white text on red background
YELLOW = "\033[1;33m"      # Bold yellow text
RESET = "\033[0m"

class Dht11Node(Node):

	def __init__(self):
		super().__init__('dht11')
		
		self.sensors = []
		for pin_num, board_pin in DHT_PINS:
			try:
				sensor = adafruit_dht.DHT11(board_pin, use_pulseio=False)
				self.sensors.append((pin_num, sensor))
			except Exception as e:
				self.get_logger().error(f"Failed to init DHT11 on GPIO {pin_num}: {e}")
				self.sensors.append((pin_num, None))

		self.temperature_publisher_ = self.create_publisher(
			Float64MultiArray, '/dht11/temperature', 10
		)
		self.humidity_publisher_ = self.create_publisher(
			Float64MultiArray, '/dht11/humidity', 10
		)

		self.initial_humidities = {}

		# DHT11 hardware minimum sampling period is 1.0s (2.0s is more reliable)
		self.timer = self.create_timer(2.0, self.timer_callback)
		self.get_logger().info(
			f"DHT11 node initialized on GPIO pins {[p for p, _ in DHT_PINS]}."
		)
		
	def timer_callback(self):
        temperatures = []
        humidities = []
        leak_warnings = []
        temp_warnings = []

        for pin_num, sensor in self.sensors:
            if sensor is None:
                temperatures.append(float('nan'))
                humidities.append(float('nan'))
                continue

            try:
                t = sensor.temperature
                h = sensor.humidity
                t_val = float(t) if t is not None else float('nan')
                h_val = float(h) if h is not None else float('nan')
                temperatures.append(t_val)
                humidities.append(h_val)

                # Track baseline X and warn only if RH >= X + 10
                if h is not None:
                    if pin_num not in self.initial_humidities:
                        self.initial_humidities[pin_num] = h_val
                        self.get_logger().info(
                            f"DHT11 GPIO {pin_num} baseline recorded: RH0 = {h_val:.1f}% | T0 = {t_val:.1f} C"
                        )
                    else:
                        rh0 = self.initial_humidities[pin_num]
                        if (h_val - rh0) >= DHT_RH_RISE_WARN:
                            leak_warnings.append(
                                f"GPIO {pin_num}: {h_val:.1f}% (+{h_val - rh0:.1f}% above {rh0:.1f}%)"
                            )

                if t is not None and t_val >= DHT_TEMP_WARN:
                    temp_warnings.append(f"GPIO {pin_num}: {t_val:.1f} C (>= {DHT_TEMP_WARN:.0f} C)")

            except RuntimeError:
                temperatures.append(float('nan'))
                humidities.append(float('nan'))
            except Exception as e:
                self.get_logger().warn(
                    f"DHT11 GPIO {pin_num} error: {e}", throttle_duration_sec=10.0
                )
                temperatures.append(float('nan'))
                humidities.append(float('nan'))

        if leak_warnings:
            self.get_logger().warn(
                f"{RED_BG} CANISTER HUMIDITY RISE (+10%): {' | '.join(leak_warnings)} {RESET}",
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
		for _, sensor in self.sensors:
			if sensor is not None:
				try:
					sensor.exit()
				except Exception:
					pass

def main(args=None):
    rclpy.init(args=args)
    node = Dht11Node()
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