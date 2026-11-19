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

		# DHT11 hardware minimum sampling period is 1.0s (2.0s is more reliable)
		self.timer = self.create_timer(2.0, self.timer_callback)
		self.get_logger().info(
			f"DHT11 node initialized on GPIO pins {[p for p, _ in DHT_PINS]}."
		)
		
	def timer_callback(self):
		temperatures = []
		humidities = []

		for pin_num, sensor in self.sensors:
			if sensor is None:
				temperatures.append(float('nan'))
				humidities.append(float('nan'))
				continue

			try:
				t = sensor.temperature
				h = sensor.humidity
				temperatures.append(float(t) if t is not None else float('nan'))
				humidities.append(float(h) if h is not None else float('nan'))
			except RuntimeError:
				temperatures.append(float('nan'))
				humidities.append(float('nan'))
			except Exception as e:
				self.get_logger().warn(
					f"DHT11 GPIO {pin_num} error: {e}", throttle_duration_sec=10.0
				)
				temperatures.append(float('nan'))
				humidities.append(float('nan'))

		temp_msg = Float64MultiArray(data=temperatures)
		hum_msg = Float64MultiArray(data=humidities)
		self.temperature_publisher_.publish(temp_msg)
		self.humidity_publisher_.publish(hum_msg)

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