import os
from dynamixel_sdk import *

class DynamixelController:
    def __init__(self, device_name='/dev/ttyUSB0', baudrate=1000000, protocol=2.0):
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(protocol)
        self.baudrate = baudrate

        self.legs_map = {
            0: [0, 1, 2],
            1: [3, 4, 5],
            2: [6, 7, 8],
            3: [9, 10, 11],
            4: [12, 13, 14],
            5: [15, 16, 17]
        }

    def connect(self):
        if not self.port_handler.openPort(): return False
        if not self.port_handler.setBaudRate(self.baudrate): return False
        return True

    def check_motor(self, motor_id):
        _, result, error = self.packet_handler.ping(self.port_handler, motor_id)
        return (result == COMM_SUCCESS and error == 0)

    def get_active_legs(self):
        active_legs = []
        
        print("Checking leg status...")
        for leg_id, motor_ids in self.legs_map.items():
            leg_status = [self.check_motor(m_id) for m_id in motor_ids]
            
            if all(leg_status):
                active_legs.append(leg_id)
                print(f"Leg {leg_id}: ONLINE (Motors {motor_ids})")
            else:
                missing = [motor_ids[i] for i, status in enumerate(leg_status) if not status]
                print(f"Leg {leg_id}: OFFLINE (Missing IDs: {missing})")
                
        return active_legs

    def disconnect(self):
        self.port_handler.closePort()

def get_hardware_status():
    controller = DynamixelController()
    if controller.connect():
        active_legs = controller.get_active_legs()
        controller.disconnect()
        return active_legs
    return []

if __name__ == "__main__":
    dxl = DynamixelController()

    if dxl.connect():
        working_legs = dxl.get_active_legs()
        print(f"\nFinal Active Leg Array: {working_legs}")
        dxl.disconnect()