import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from smbus2 import SMBus, i2c_msg
from time import sleep

MS8607_ADDR = 0x76
MS8607_ADDR_HUMIDITY = 0x40
RESET_OFF = 0x1E

# Relative humidity rise threshold (%) above startup baseline to flag a potential leak
RH_RISE_WARN_THRESHOLD = 5.0

class Ms8607Node(Node):
    
    def __init__(self, bus=0):        
        super().__init__(f'ms8607_i2c{bus}')
        self.bus_num = bus

        try:
            self._bus = SMBus(bus)
        except Exception as e:
            self.get_logger().error(f"Bus i2c-{bus} is not available: {e}")
            raise

        # Reset pressure/temp & humidity sensors once at startup
        self._bus.write_byte(MS8607_ADDR, RESET_OFF)
        self._bus.write_byte(MS8607_ADDR_HUMIDITY, 0xFE)
        sleep(0.05)

        # Read 12 bytes of factory calibration (SENST1, OFFT1, TCS, TCO, TREF, TEMPSENS)
        data1 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xA2, 2)
        data2 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xA4, 2)
        data3 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xA6, 2)
        data4 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xA8, 2)
        data5 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xAA, 2)
        data6 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xAC, 2)
        
        self.c1 = (data1[0] << 8) | data1[1]
        self.c2 = (data2[0] << 8) | data2[1]
        self.c3 = (data3[0] << 8) | data3[1]
        self.c4 = (data4[0] << 8) | data4[1]
        self.c5 = (data5[0] << 8) | data5[1]
        self.c6 = (data6[0] << 8) | data6[1]

        self.initial_humidity = None

        prefix = f'/ms8607_i2c{bus}'
        self.pressure_publisher_ = self.create_publisher(Float64, f'{prefix}/pressure', 10)
        self.temperature_publisher_ = self.create_publisher(Float64, f'{prefix}/temperature', 10)
        self.humidity_publisher_ = self.create_publisher(Float64, f'{prefix}/humidity', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info(f"Sensor MS8607 on bus i2c-{bus} correctly initialized.")

    def timer_callback(self):
        try:
            pressure, temp = self.get_pressure_and_temperature()
            humidity = self.get_humidity()

            if self.initial_humidity is None:
                self.initial_humidity = humidity
                self.get_logger().info(
                    f"Canister baseline recorded: RH0 = {self.initial_humidity:.2f}% | "
                    f"T0 = {temp:.2f} C | P0 = {pressure:.1f} mbar"
                )
            elif (humidity - self.initial_humidity) >= RH_RISE_WARN_THRESHOLD:
                self.get_logger().warn(
                    f"CANISTER HUMIDITY RISING: {humidity:.2f}% "
                    f"(+{humidity - self.initial_humidity:.2f}% above baseline {self.initial_humidity:.2f}%)! "
                    f"Check for water ingress.",
                    throttle_duration_sec=5.0,
                )

            self.pressure_publisher_.publish(Float64(data=pressure))
            self.temperature_publisher_.publish(Float64(data=temp))
            self.humidity_publisher_.publish(Float64(data=humidity))
        except Exception as e:
            self.get_logger().warn(f"MS8607 read error: {e}", throttle_duration_sec=5.0)
    
    def get_pressure_and_temperature(self):
        # 0x40: D1 conversion (OSR=256)
        self._bus.write_byte(MS8607_ADDR, 0x40)
        sleep(0.005)
        d1_bytes = self._bus.read_i2c_block_data(MS8607_ADDR, 0x00, 3)
        D1 = (d1_bytes[0] << 16) | (d1_bytes[1] << 8) | d1_bytes[2]

        # 0x50: D2 conversion (OSR=256)
        self._bus.write_byte(MS8607_ADDR, 0x50)
        sleep(0.005)
        d2_bytes = self._bus.read_i2c_block_data(MS8607_ADDR, 0x00, 3)
        D2 = (d2_bytes[0] << 16) | (d2_bytes[1] << 8) | d2_bytes[2]

        dT = D2 - self.c5 * 256
        Temp = 2000 + dT * self.c6 / 8388608.0
        OFF = self.c2 * 131072 + (self.c4 * dT) / 64.0
        SENS = self.c1 * 65536 + (self.c3 * dT) / 128.0

        if Temp >= 2000:
            Ti = 5 * (dT * dT) / 274877906944.0
            OFFi = 0.0
            SENSi = 0.0
        else:
            Ti = 3 * (dT * dT) / 8589934592.0
            OFFi = 61 * ((Temp - 2000) ** 2) / 16.0
            SENSi = 29 * ((Temp - 2000) ** 2) / 16.0
            if Temp < -1500:
                OFFi += 17 * ((Temp + 1500) ** 2)
                SENSi += 9 * ((Temp + 1500) ** 2)

        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi

        cTemp = (Temp - Ti) / 100.0
        pressure = ((((D1 * SENS2) / 2097152.0) - OFF2) / 32768.0) / 100.0
        return float(pressure), float(cTemp)
    
    def get_humidity(self):
        # 0xF5: Measure RH No Hold Master
        self._bus.write_byte(MS8607_ADDR_HUMIDITY, 0xF5)
        sleep(0.025)
        read_msg = i2c_msg.read(MS8607_ADDR_HUMIDITY, 3)
        self._bus.i2c_rdwr(read_msg)
        buf = list(read_msg)
        D3 = ((buf[0] << 8) | buf[1]) & 0xFFFC
        humidity = -6.0 + (125.0 * (D3 / 65536.0))
        return float(humidity)

def main(args=None):
    rclpy.init(args=args)
    node = Ms8607Node(bus=0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()