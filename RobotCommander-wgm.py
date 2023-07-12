####### Command for control conveyor #################
#### activate tcp        = activate,tcp
#### power on servo      = pwr_on,conv,0
#### power off servo     = pwr_off,conv,0
#### set velocity x mm/s = set_vel,conv,x   # x = 0 to 200
#### jog forward         = jog_fwd,conv,0
#### jog backward        = jog_bwd,conv,0
#### stop conveyor       = jog_stop,conv,0
#########################################################

import socket
import time
import cv2
import math
import numpy as np


# the IP address of the UR arm
ROBOT_IP = '10.10.0.14'
# port used by UR arm
ARM_PORT = 30003
# arbeitely chosen move time (in s)
MOVE_TIME = 0.03
# the starting pose of the gripper (in [m, m, m, rad, rad, rad])
HOME_POSE = [0.0662, -0.4362, 0.5636, 0.0, 0.0, 0.0]# the starting pose of the gripper (in [m, m, m, rad, rad, rad])
CHANGE_POSE = HOME_POSE.copy()
CHANGE_POSE[3]= math.radians(10)
CHANGE_POSE[4]= math.radians(10)
# the buffer size
BUFFER_SIZE = 255

class RobotCommander():

    def __init__(self):
        self._arm_connection = None
        self._gripper_connection = None

        self._establish_arm_connection()


    def _establish_arm_connection(self):
        self._arm_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._arm_connection.connect((ROBOT_IP, ARM_PORT))
        recv = self._arm_connection.recv(BUFFER_SIZE)
        if recv:
                print('succesfully connected to the arm')
        else:
                print('failed to connect to the arm')

    def movel(self, target_pose, a=1.2, v=0.25, t=0, r=0):
        cmd_str = f"movel(p{target_pose},{a},{v},{t},{r})\n"
        print(cmd_str)
        self._arm_connection.send(bytes(cmd_str, "UTF-8"))
        print("done")
        

   
if __name__ == '__main__':
    print("Running RobotCommander")
    robot_commander = RobotCommander()
    #Initalizing
    for i in range(10):
        robot_commander.movel(HOME_POSE,v=1)
        robot_commander.movel(CHANGE_POSE,v=1)
        robot_commander.movel(HOME_POSE,v=1)