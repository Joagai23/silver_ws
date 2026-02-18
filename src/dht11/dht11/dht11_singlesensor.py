import rclpy
from rclpy.node import Node
from silver_interfaces.msg import DhtSensorData  # Import your custom message
import adafruit_dht
import board
import time

class DHT11SensorNode(Node):
    def __init__(self):
        super().__init__('dht11_sensor_node')

        self.declare_parameter('gpio_pin', 4)  # Default to GPIO 4
        self.declare_parameter('sensor_location', 'Unknown')
        self.declare_parameter('sensor_type', 'DHT11')

        self.gpio_pin_param = self.get_parameter('gpio_pin').value # Renamed to avoid conflict if self.gpio_pin was meant for board pin
        self.sensor_location = self.get_parameter('sensor_location').value
        self.sensor_type = self.get_parameter('sensor_type').value

        # Mapping from integer GPIO numbers (BCM) to board pins
        # This mapping assumes you are passing BCM numbers as parameters
        # and want to use them with adafruit_dht which expects board.D# pins.
        # If self.gpio_pin_param is already a board.D# constant, this mapping needs adjustment.
        # For simplicity, this example assumes sensor_location is key to board.D# pin.
        
        # Direct mapping from location to board pins
        # Ensure these locations match what you pass as 'sensor_location' parameter
        gpio_board_pins = {
            'F4': board.D4, 'T1': board.D5, 'F1': board.D6, 'F3': board.D7, # Note: Original list had F3 then F4, check your wiring
            'C5': board.D9, 'F5': board.D10, 'ControlC': board.D11, 'CameraC': board.D13,
            'C4': board.D14, 'T3': board.D15, 'T4': board.D16, 'T2': board.D17, # Note: Original list had C4, T3, T4, T2 in different order for pins 14-17
            'C3': board.D20, 'T5': board.D21, 'C2': board.D22, 'T0': board.D23,
            'F0': board.D24, 'C0': board.D25, 'C1': board.D26, 'F2': board.D27
        }
        # Original lists for reference if needed for debugging sensor_gpio_mapping logic
        # gpio_pins_orig_order = [
        #     board.D4, board.D5, board.D6, board.D7, board.D9, board.D10, board.D11, board.D13,
        #     board.D14, board.D15, board.D16, board.D17, board.D20, board.D21, board.D22, board.D23,
        #     board.D24, board.D25, board.D26, board.D27
        # ]
        # sensor_locations_orig_order = [
        #     'F3', 'T1', 'F1', 'F4', 'C5', 'F5', 'ControlC', 'CameraC', 'C3', 'T4', 'T3', 'T2',
        #     'C4', 'T5', 'C2', 'T0', 'F2', 'C0', 'C1', 'F0'
        # ]
        # sensor_gpio_mapping = dict(zip(sensor_locations_orig_order, gpio_pins_orig_order))
        
        target_board_pin = gpio_board_pins.get(self.sensor_location)

        if target_board_pin is None:
            self.get_logger().error(
                f"Sensor location '{self.sensor_location}' not found in pin mapping. "
                f"Ensure it's one of {list(gpio_board_pins.keys())}. Node will not function correctly."
            )
            # Optionally, raise an exception or handle this more gracefully
            # For now, dhtDevice might be None or initialization might fail
            self.dhtDevice = None
        else:
            self.get_logger().info(f"Initializing DHT sensor type '{self.sensor_type}' at location '{self.sensor_location}' on pin {target_board_pin}")
            if self.sensor_type == 'DHT11':
                self.dhtDevice = adafruit_dht.DHT11(target_board_pin)
            elif self.sensor_type == 'DHT22':
                self.dhtDevice = adafruit_dht.DHT22(target_board_pin)
            else:
                self.get_logger().error(f"Unsupported sensor type: {self.sensor_type}. Defaulting to DHT11.")
                self.dhtDevice = adafruit_dht.DHT11(target_board_pin)


        # Create publisher for the custom message
        self.sensor_data_publisher = self.create_publisher(DhtSensorData, 'dht11_sensor_data', 10)

        # Timer to read the sensor data periodically
        self.timer = self.create_timer(2.0, self.new_read_sensor_data)  # 2-second interval

        self.get_logger().info(f"DHT11 sensor node for location '{self.sensor_location}' has started.")

    def new_read_sensor_data(self):

        if self.dhtDevice is None:
            self.get_logger().error(f"DHT device for location {self.sensor_location} not initialized. Cannot read sensor.")
            return

        try:
            # Attempt to get temperature and humidity
            temperature_c = self.dhtDevice.temperature
            humidity = self.dhtDevice.humidity

            if temperature_c is not None and humidity is not None:
                """self.get_logger().info(
                    "Sensor {}: Temp: {:.1f} C    Humidity: {}% ".format(
                        self.sensor_location, temperature_c, humidity
                    )
                )"""
                # Publish the custom message
                sensor_data_msg = DhtSensorData()
                sensor_data_msg.location = self.sensor_location
                sensor_data_msg.temperature = float(temperature_c)
                sensor_data_msg.humidity = float(humidity)
                self.sensor_data_publisher.publish(sensor_data_msg)
            else:
                # This case handles when the library itself returns None, indicating a read failure
                """self.get_logger().warning(
                    f"Failed to read data from DHT sensor at {self.sensor_location} (got None). "
                    "Check wiring and pull-up resistor. Will retry on next timer tick."
                )"""

        except RuntimeError as error:
            # Errors from the DHT library (e.g., checksum mismatch, timeout)
            # These are fairly common with DHT sensors.
            """self.get_logger().warning(
                f"RuntimeError reading sensor {self.sensor_location}: {error.args[0]}. "
                "Will retry on next timer tick."
            )"""
            # The original code had `print(error.args[0])` and `time.sleep(2.0)` and `continue`
            # within a `while True` loop. The ROS timer handles retries now.

        except Exception as error:
            # For any other unexpected errors, follow the "as before" instruction.
            self.get_logger().error(f"Unexpected critical error reading sensor {self.sensor_location}: {error}. Exiting DHT device and re-raising.")
            if self.dhtDevice: # Check if dhtDevice exists before calling exit
                self.dhtDevice.exit()
            raise error # Re-raising the error will likely stop this timer callback or the node.

def main(args=None):
    rclpy.init(args=args)

    node = DHT11SensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Node {node.get_name()} shutting down due to KeyboardInterrupt.")
    except Exception as e: # Catch exceptions from re-raise in timer callback
        node.get_logger().fatal(f"Unhandled exception in {node.get_name()}: {e}. Shutting down.")
    finally:
        if rclpy.ok(): # Check if context is still valid
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

