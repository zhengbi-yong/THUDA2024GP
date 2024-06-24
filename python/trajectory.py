import numpy as np
import time
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import json
import os
class LeapNode:
    def __init__(self):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.prev_pos = self.pos = lhu.allegro_to_LEAPhand(np.zeros(16))
           
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
            self.curr_pos = self.dxl_client.read_pos()
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
    
    def control_joint_by_input(self, joint_id, degree):
        if 0 <= joint_id < len(self.motors):
            # 直接将度转换为弧度，因为write_desired_pos内部会处理转换
            radians = np.radians(degree)
            # 更新self.curr_pos以反映指定关节的新位置（弧度）
            self.curr_pos[joint_id] = radians
            # 调用write_desired_pos更新舵机位置
            self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        else:
            print("Invalid joint ID. Please enter a number between 0 and 15.")
            
    def move_to_pose(self, target_pose, duration=5.0):
        """Moves to the given pose in the specified duration.

        Args:
            target_pose: Target joint angles in radians.
            duration: Duration in seconds to reach the target pose.
        """
        current_pose = self.dxl_client.read_pos()  # 当前姿态
        steps = 100  # 分成100步完成动作
        step_duration = duration / steps  # 每步持续时间

        # 计算每步的增量
        pose_increment = (target_pose - current_pose) / steps

        for step in range(steps):
            # 计算新的姿态
            new_pose = current_pose + pose_increment * (step + 1)
            self.dxl_client.write_desired_pos(self.motors, new_pose)
            time.sleep(step_duration)
            
    def move_to_poses_from_json(self, json_file_name):
        """Reads joint states and durations from a JSON file and executes them sequentially."""
        # 获取当前脚本文件的目录
        current_dir = os.path.dirname(__file__)
        # 构建json文件的完整路径
        json_file_path = os.path.join(current_dir, json_file_name)
        
        with open(json_file_path, 'r') as file:
            states = json.load(file)
        
        for state in states:
            pose = np.array(state["pose"])
            duration = state["duration"]
            self.move_to_pose(pose, duration=duration)



def main(**kwargs):
    leap_hand = LeapNode()
    json_file_path = './trajectory/test.json'  # 修改为你的JSON文件路径
    leap_hand.move_to_poses_from_json(json_file_path)

if __name__ == "__main__":
    main()