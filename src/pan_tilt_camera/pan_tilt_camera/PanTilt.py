import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gpiozero import Servo

class PanTilt(Node):

    def __init__(self):
        super().__init__('pan_tilt_control')

        self.pan_PIN = 12
        self.tilt_PIN = 18

        # Tuned pulse widths to reduce jitter
        self.pan = Servo(
            self.pan_PIN,
            min_pulse_width=0.0006,  # 0.6 ms
            max_pulse_width=0.0024   # 2.4 ms
        )
        self.tilt = Servo(
            self.tilt_PIN,
            min_pulse_width=0.0006,
            max_pulse_width=0.0024
        )

        self.subscription1 = self.create_subscription(
            Float64,
            'pan_value',
            self.pan_callback,
            10)

        self.subscription2 = self.create_subscription(
            Float64,
            'tilt_value',
            self.tilt_callback,
            10)

        self.get_logger().info('Camera Pan and Tilt Control node initialized with gpiozero.Servo.')

    def pan_callback(self, msg):
        servo_value = self.clamp_servo_value(msg.data)
        self.pan.value = servo_value
        self.get_logger().debug(f'Pan set to: {servo_value:.2f}')

    def tilt_callback(self, msg):
        servo_value = self.clamp_servo_value(msg.data)
        self.tilt.value = servo_value
        self.get_logger().debug(f'Tilt set to: {servo_value:.2f}')

    def clamp_servo_value(self, value):
        if value < -1.0 or value > 1.0:
            self.get_logger().warn(f'Servo input {value:.2f} out of range, clamping to [-1, 1]')
        return max(-1.0, min(1.0, value))

    def destroy_node(self):
        # Stop PWM signals to prevent jitter
        self.pan.detach()
        self.tilt.detach()
        self.get_logger().info('PWM signals detached from servos.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    pan_tilt_control = PanTilt()

    try:
        rclpy.spin(pan_tilt_control)
    except KeyboardInterrupt:
        pass
    finally:
        pan_tilt_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
