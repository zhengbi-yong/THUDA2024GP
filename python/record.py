import numpy as np
import time
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu

class LeapHandReader:
    def __init__(self):
        self.motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        # 尝试连接到不同的端口，直到成功
        try:
            self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(self.motors, 'COM13', 4000000)
                self.dxl_client.connect()
        
        # 关闭所有电机的扭矩，以允许手动操作
        self.dxl_client.set_torque_enabled(self.motors, False)

    def read_and_print_positions(self):
        # 循环读取并打印所有关节的位置
        try:
            while True:
                positions = self.dxl_client.read_pos()
                print("Current Positions:", list(positions))
                time.sleep(1)  # 每秒读取一次位置
        except KeyboardInterrupt:
            print("Program terminated by user.")

def main():
    leap_hand_reader = LeapHandReader()
    leap_hand_reader.read_and_print_positions()

if __name__ == "__main__":
    main()