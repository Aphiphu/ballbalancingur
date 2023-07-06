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
CHANGE_POSE = HOME_POSE.copy()

# Constants for plate position
set_y = 220
set_x = 250
max_y = 475
min_y = 1.5
max_x = 422
min_x = 12
# array to save change in angle
y_rad_change=[0,0]
# PID controller
count = 0 
pixel_1m_y = (max_y-min_y)/0.2
pixel_1m_x = (max_x-min_x)/0.13
Kc_y = 0.8/pixel_1m_y
Ki_y = 0.15/pixel_1m_y
Kd_y = 2.2/pixel_1m_y

Kc_x = 0.8/pixel_1m_x
Ki_x = 0.15/pixel_1m_x
Kd_x = 2.2/pixel_1m_x
time_step = 0.03
integral = [0.0,0.0]
derivative = [0.0,0.0]
#Store variable for ploting
t=np.array([])
xs = np.array([])
ys = np.array([])
start=[]
angle_stored_x = []
angle_stored_y = []
e = np.array([])
# pandas dataframe for collecting data
df=pd.DataFrame()
# pixel offset (in pixels)
pixel_off_x = 10
pixel_off_y = 10
# the buffer size
BUFFER_SIZE = 255

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

def calc_angle(pos, i):
    global integral, derivative, time_step, xs, ys, t, e 
    t = np.append(t,time.time()-t0)
    x0 = pos[0]
    y0 = pos[1]
    xs = np.append(xs,x0)
    ys = np.append(ys,y0)
    error = [x0-set_x, y0-set_y]
    e = np.append(e,error)             # the error, use x0-SP for correct sign of angle
    if i >=1:
        derivative[0] = (xs[i]-xs[i-1])/ time_step
        derivative[1] = (ys[i]-ys[i-1])/ time_step
    integral[0] += error[0]*time_step
    integral[1] += error[1]*time_step
    
    # PID eq
    angle_x = (-Kc_x *error[0] - Kd_x*derivative[0] - Ki_x*integral[0])/1.5
    angle_y = (-Kc_y *error[1] - Kd_y*derivative[1] - Ki_y*integral[1])/1.5
    # Limit movement of arm (in rad) 
    max_angle = math.radians(1)
    if angle_x > max_angle:
        angle_x = max_angle
    if angle_x < -max_angle:
        angle_x = -max_angle
    if angle_y > max_angle:
        angle_y = max_angle
    if angle_y < -max_angle:
        angle_y = -max_angle
    # on the spot checking
    print("Error", error)
    print("Derivative", derivative)
    print("Integral", integral)
    angle_stored_x.append(angle_x)
    angle_stored_y.append(angle_y)
    return angle_x,angle_y
         

        
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
    t1 = time.time()
    start.append(t1)
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
            print(f"x:{x},y:{y}")            
            rad_change = calc_angle([x,y], count)
            count+=1
            print("Detected count:", count)
            CHANGE_POSE[3]=-rad_change[0]
            if (abs(x-set_x)<pixel_off_x): CHANGE_POSE[3]=0
            CHANGE_POSE[4]=rad_change[1]
            if (abs(y-set_y)<pixel_off_y): CHANGE_POSE[4]=0
            robot_commander.movel(CHANGE_POSE, t=MOVE_TIME)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            ['cur_x','cur_y','error','derivative','integral','angle_x','angle_y','time']
            df['cur_x']=xs
            df['cur_y']=ys
            df['error']=e
            df['derivative']=derivative
            df['integral']=integral
            df['angle_x']=angle_stored_x
            df['angle_y']=angle_stored_y
            df['time']=time
            break
    cap.release()
    cv2.destroyAllWindows()
