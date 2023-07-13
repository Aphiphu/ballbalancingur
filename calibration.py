import socket
import time
import cv2
import csv
import math
import numpy as np
import pandas as pd

# the IP address of the UR arm
ROBOT_IP = '10.10.0.14'
# port used by UR arm
ARM_PORT = 30003
# arbeitely chosen move time (in s)
MOVE_TIME = 0.3
# port used by robotiq gripper
GRIPPER_PORT = 63352
# the starting pose of the gripper (in [m, m, m, rad, rad, rad])
HOME_POSE = [0.0662, -0.4362, 0.5636, 0.0, 0.0, 0.0]# the starting pose of the gripper (in [m, m, m, rad, rad, rad])
# the buffer size
BUFFER_SIZE = 255
# set point
PIXEL_TO_MM =1
set_x = 250
set_y = 220

class RobotCommander():
    
    def __init__(self):
        #set arm and gripper connection variables
        self._arm_connection = None
        self._gripper_connection = None
        # run establishing functions for arm and gripper
        self._establish_arm_connection()
        self._establish_gripper_connection()
        
            
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
        
    def movel(self, target_pose, a=1.2, v=0.25, t=0, r=0):
        cmd_str = f"movel(p{target_pose},{a},{v},{t},{r})\n"
        print(cmd_str)
        self._arm_connection.send(bytes(cmd_str, "UTF-8"))
        
    def grip(self):
        self._gripper_connection.send(b'SET POS 255\n')
        
    def grip_release(self):
        self._gripper_connection.send(b'SET POS 0\n')
    
def measurepixeltomm(x_measured, y_measured, x_real, y_real):
    PIXEL_TO_MM_X = x_real/x_measured
    PIXEL_TO_MM_Y = y_real/y_measured
    PIXEL_TO_MM = (PIXEL_TO_MM_X+PIXEL_TO_MM_Y)/2
    return PIXEL_TO_MM

if __name__ == '__main__':
    robot_commander = RobotCommander()
    #Initalizing
    robot_commander.grip()
    robot_commander.movel(HOME_POSE, t=MOVE_TIME)
    #time.sleep(MOVE_TIME)
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # 0 indicates the default camera
    width = cap.get(3)
    height = cap.get(4)
    print("the width of the frame is ", width)  # x_axis
    print("the height of the frame is ",height) # y_axis
    while (cap.isOpened()):
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([14,100,100])
        upper_orange = np.array([21,255,255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.circle(frame,(set_x,set_y),20,(255,255,255),2)
        if len(contours) > 0:
            t0 = time.time()
            c = max(contours, key=cv2.contourArea)
            (x,y),radius = cv2.minEnclosingCircle(c)
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(frame,(set_x,set_y),20,(255,255,255),2)
            cv2.circle(frame,center,radius,(0,255,0),2)
            cv2.circle(frame,center,radius=5, color=(0, 0, 255), thickness=-1)
            print(f"ball_x:{x},ball_y:{y}")            