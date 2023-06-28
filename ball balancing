####### Command for control conveyor #################
#### activate tcp        = activate,tcp
#### power on servo      = pwr_on,conv,0
#### power off servo     = pwr_off,conv,0
#### set velocity x mm/s = set_vel,conv,x   # x = 0 to 200
#### jog forward         = jog_fwd,conv,0
#### jog backward        = jog_bwd,conv,0
#### stop conveyor       = jog_stop,conv,0
#########################################################

import sys
import socket
import time
import binascii
import numpy as np

# IP address of the camera 
CAM_IP = "10.10.1.10"
# port used by the camera
CAM_PORT = 2023
# the IP address of the UR arm
ROBOT_IP = '10.10.0.14'
# port used by UR arm
ARM_PORT = 30003
# port used by robotiq gripper
GRIPPER_PORT = 63352
# the height from home position to the grabbing height (in m)
#need adjustment, depends on the height of the box
#HEIGHT = 0.145 for 0.2
HEIGHT = 0.11
BASE_CONVEYOR_OFFSET = 0.320
# the offset distance between the gripper to the camera (in m)
BASE_CAM_OFFSET = 0.315
# the middle of the image with respect to the camera frame origin (in m)
MIDDLE = [0.052, -0.044]
# the time the arm takes to move from any point A to any point B (in s)
MOVE_TIME = 0.50
# the time the gripper take to grip an object firmly (in s)
GRIPPING_TIME = 0.35
# the starting pose of the gripper (in [m, m, m, rad, rad, rad])
HOME_POSE = [0.116, -0.325, 0.165, 0.0, -3.143, 0.0]
# the pose when placing object off the running conveyor belt
PLACING_POSE = [-0.350, -0.325, -0.040, 0.0, 3.181, 0.0]
# the speed of the conveyor (in m/s)
CONVEYOR_SPEED = 0.01
# the average detection delay (in s)
# currently not known, requires fine tuning
DETECTION_DELAY = 1.0
# the buffer size
BUFFER_SIZE = 255

class RobotCommander():
    
    def __init__(self):
        self._cam_connection = None
        self._arm_connection = None
        self._gripper_connection = None
        
        self._establish_cam_connection()
        self._establish_arm_connection()
        self._establish_gripper_connection()
        
    def _establish_cam_connection(self):
        self._cam_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._cam_connection.connect((CAM_IP, CAM_PORT))
        self._cam_connection.send(b'GET ACT\n')
        recv = str(self._cam_connection.recv(BUFFER_SIZE), 'UTF-8')
        if '1' in recv:
             print('succesfully connected to the camera')
        else:
             print('failed to connect to the camera')
            
    def _establish_arm_connection(self):
        self._arm_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._arm_connection.connect((ROBOT_IP, ARM_PORT))
        recv = self._arm_connection.recv(BUFFER_SIZE)
        if recv:
                print('succesfully connected to the arm')
        else:
                print('failed to connect to the arm')
                
    def _establish_gripper_connection(self):
        self._gripper_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._gripper_connection.connect((ROBOT_IP, GRIPPER_PORT))
        # recv = self._gripper_connection.recv(BUFFER_SIZE)
        # if recv:
        #         print('succesfully connected to the gripper')
        #         self.grip_release()
        # else:
        #         print('failed to connect to the gripper')
        
    def movel(self, target_pose, a=1.2, v=0.25, t=0, r=0):
        cmd_str = f"movel(p{target_pose},{a},{v},{t},{r})\n"
        print(cmd_str)
        self._arm_connection.send(bytes(cmd_str, "UTF-8"))

    def movej(self, target_joint_params, a=1.2, v=0.25, t=0, r=0):
        cmd_str = f"movel({target_joint_params},{a},{v},{t},{r})\n"
        print(cmd_str)
        self._arm_connection.send(bytes(cmd_str, "UTF-8"))
        
    def grip(self):
        self._gripper_connection.send(b'SET POS 255\n')
        
    def grip_release(self):
        self._gripper_connection.send(b'SET POS 0\n')
        
    def get_cam_detection(self):
        recv = str(self._cam_connection.recv(BUFFER_SIZE), 'UTF-8')
        coordinates = recv[1:-1].replace(")(",",").split(",")
        coordinates = ["nan" if (i == "" or i == " " ) else i for i in coordinates]
        x_cam = float(coordinates[0]) / 1000
        y_cam = float(coordinates[1]) / 1000
        rz_cam = float(coordinates[2]) / 1000
        return x_cam, y_cam, rz_cam


if __name__ == '__main__':
    robot_commander = RobotCommander()
    #Initalizing
    robot_commander.movel(HOME_POSE, t=MOVE_TIME)
    time.sleep(MOVE_TIME)
    while True:
        x_cam, y_cam, rz_cam = robot_commander.get_cam_detection()
        print(x_cam, y_cam, rz_cam)
        #if not (np.isnan(x_cam) or np.isnan(y_cam) or np.isnan(rz_cam)):
        if not (np.isnan(x_cam) or np.isnan(y_cam)):
            print("an object is detected, starting the move sequence")
            print("moving to the object")
            x_base = (y_cam - MIDDLE[1]) + BASE_CAM_OFFSET - \
                CONVEYOR_SPEED*(MOVE_TIME + DETECTION_DELAY)
            y_base = (x_cam - MIDDLE[0]) - BASE_CONVEYOR_OFFSET
            robot_commander.grip_release()
            robot_commander.movel([x_base, y_base, -HEIGHT, 0, -3.142, 0], t=MOVE_TIME)
            time.sleep(MOVE_TIME)
            print("grabbing the object")
            robot_commander.grip()
            time.sleep(GRIPPING_TIME)
            print("lifting the object back to home position")
            robot_commander.movel(HOME_POSE, t=MOVE_TIME)
            time.sleep(MOVE_TIME + 1)
            print("moving to the placing position")
            robot_commander.movel(PLACING_POSE, t=MOVE_TIME+2)
            time.sleep(MOVE_TIME + 2)
            print("releasing the object")
            robot_commander.grip_release()
            time.sleep(GRIPPING_TIME)
            print("moving back to home position")
            robot_commander.movel(HOME_POSE, t=MOVE_TIME)
            time.sleep(MOVE_TIME)
            print("move sequence done")
            print("====="*16)
