import numpy as np
import time
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu

class LeapNode:
    def __init__(self):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
           
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM13', 4000000)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position
    def read_pos(self):
        return self.dxl_client.read_pos()
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
    

def gradual_movement(leap_hand, target_position, duration):
    """
    Gradually moves the hand to a target position within a given duration.
    """
    start_position = leap_hand.read_pos()  # Read current position
    steps = int(duration / 0.03)  # Calculate steps
    for step in range(steps + 1):
        alpha = step / steps
        current_position = (1 - alpha) * start_position + alpha * target_position
        leap_hand.dxl_client.write_desired_pos(leap_hand.motors, current_position)
        time.sleep(0.03)

# def main(**kwargs):
#     leap_hand = LeapNode()
#     # Assuming the initial position is 0 for all motors
#     original_position = np.zeros(16)
    
#     # Calculate the target position in ticks for 1 radian
#     ticks_per_radian = 4096 / (2 * np.pi)
#     target_position = original_position + np.ones(16) * (0.05)  # For 1 radian

#     while True:
#         # Move to the target position over 5 seconds
#         gradual_movement(leap_hand, target_position, 1)
#         time.sleep(1)  # Keep the position for 5 seconds
#         # Return to the original position over 5 seconds
#         gradual_movement(leap_hand, original_position, 1)
#         time.sleep(1)  # Keep the position for 5 seconds
def main(**kwargs):
    leap_hand = LeapNode()
    while True:
        # 创建一个全为0的数组，然后将0号关节的值设置为5度
        allegro_positions = np.zeros(16)
        allegro_positions[0] = -30 *3.14/180 # 假设这里5代表5度，如果需要转换为弧度，则使用 np.radians(5)
        
        leap_hand.set_allegro(allegro_positions)
        
        print("Position: " + str(leap_hand.read_pos()))
        time.sleep(0.03)

if __name__ == "__main__":
    main()