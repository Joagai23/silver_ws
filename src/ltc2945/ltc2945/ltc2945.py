import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from smbus2 import SMBus


LTC2945_ADDR = 103  # 0x67 on i2c-0

# Register offsets & lengths
POWER_OFF = 0x05
POWER_NBYTES = 3
DELTA_SENSE_OFF = 0x14
DELTA_SENSE_NBYTES = 2
VIN_OFF = 0x1E
VIN_NBYTES = 2
ADIN_OFF = 0x28
ADIN_NBYTES = 2

# Hardware calibration constants (R_shunt = 0.003 Ohm, 5W)
R_SHUNT = 0.003                         # Ohms
V_SENSE_LSB = 25.0e-6                   # 25 uV per LSB (102.4 mV full-scale / 4096)
VIN_RES = 0.025                         # 25 mV per LSB (102.4 V full-scale / 4096)
ADIN_RES = 0.5e-3                       # 0.5 mV per LSB
CURRENT_RES = V_SENSE_LSB / R_SHUNT     # 8.333333e-3 A/LSB (~8.33 mA/LSB)
POWER_RES = CURRENT_RES * VIN_RES       # 2.083333e-4 W/LSB

# 4S6P Li-Ion Battery Specs (18 Ah, 266.4 Wh)
BATT_V_MAX = 16.8                       # 4.2 V/cell (100%)
BATT_V_NOM = 14.8                       # 3.7 V/cell
BATT_V_WARN = 13.2                      # 3.3 V/cell (~15% remaining warning)
BATT_V_MIN_SAFE = 12.0                  # 3.0 V/cell (CRITICAL: Vicor DC/DC won't cut off!)
BATT_I_CONT_MAX = 60.0                  # 60 A max continuous discharge

class Ltc2945Node(Node):
    
    def __init__(self, bus=0):        
        super().__init__('ltc2945')

        try:
            self._bus = SMBus(bus)
        except Exception as e:
            self.get_logger().error(f"Bus i2c-{bus} is not available: {e}")
            raise

        self.vin_publisher_ = self.create_publisher(Float64, '/ltc2945/vin', 10)
        self.power_publisher_ = self.create_publisher(Float64, '/ltc2945/power', 10)
        self.current_publisher_ = self.create_publisher(Float64, '/ltc2945/current', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info(
            f"Sensor LTC2945 on bus i2c-{bus} (0x{LTC2945_ADDR:02x}) initialized | "
            f"R_shunt={R_SHUNT} Ohm | 4S6P cutoff={BATT_V_MIN_SAFE}V"
        )

    def timer_callback(self):
        try:
            vin = self.get_vin()
            current = self.get_current()
            power = self.get_power()

            vin_msg = Float64(data=float(vin))
            current_msg = Float64(data=float(current))
            power_msg = Float64(data=float(power))

            self.vin_publisher_.publish(vin_msg)
            self.current_publisher_.publish(current_msg)
            self.power_publisher_.publish(power_msg)

            # 4S6P Li-Ion under-voltage protection alerts
            if vin <= BATT_V_MIN_SAFE:
                self.get_logger().fatal(
                    f"CRITICAL BATTERY VOLTAGE: {vin:.2f} V <= {BATT_V_MIN_SAFE:.1f} V! "
                    f"Vicor DC/DC will NOT cut off until ~8V. SHUT DOWN IMMEDIATELY!",
                    throttle_duration_sec=2.0,
                )
            elif vin <= BATT_V_WARN:
                self.get_logger().warn(
                    f"LOW BATTERY WARNING: {vin:.2f} V (Safe minimum is {BATT_V_MIN_SAFE:.1f} V)",
                    throttle_duration_sec=10.0,
                )

            if current >= BATT_I_CONT_MAX:
                self.get_logger().warn(
                    f"HIGH CURRENT WARNING: {current:.2f} A >= {BATT_I_CONT_MAX:.1f} A continuous limit!",
                    throttle_duration_sec=2.0,
                )

        except Exception as e:
            self.get_logger().warn(f"LTC2945 read error: {e}", throttle_duration_sec=5.0)
    
    def get_adin(self) -> float:
        data = self._bus.read_i2c_block_data(LTC2945_ADDR, ADIN_OFF, ADIN_NBYTES)
        raw_12bit = (data[0] << 4) | (data[1] >> 4)
        return raw_12bit * ADIN_RES

    def get_current(self) -> float:
        data = self._bus.read_i2c_block_data(LTC2945_ADDR, DELTA_SENSE_OFF, DELTA_SENSE_NBYTES)
        raw_12bit = (data[0] << 4) | (data[1] >> 4)
        return raw_12bit * CURRENT_RES
    
    def get_power(self) -> float:
        data = self._bus.read_i2c_block_data(LTC2945_ADDR, POWER_OFF, POWER_NBYTES)
        raw_24bit = (data[0] << 16) | (data[1] << 8) | data[2]
        return raw_24bit * POWER_RES

    def get_vin(self) -> float:
        data = self._bus.read_i2c_block_data(LTC2945_ADDR, VIN_OFF, VIN_NBYTES)
        raw_12bit = (data[0] << 4) | (data[1] >> 4)
        return raw_12bit * VIN_RES

def main(args=None):
    rclpy.init(args=args)
    node = Ltc2945Node(bus=0)
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