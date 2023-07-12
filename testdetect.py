import cv2
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import time
t0 = time.time()
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW) # 0 indicates the default camera
#Get the video frame size
width = cap.get(3) # x_axis
height = cap.get(4)  # ya_xis
print("the width of the frame is ", width)  # x_axis
print("the height of the frame is ",height) # y_axis
t = np.array([])
r = np.array([])
set_y = 220
set_x = 250
max_y = 475
min_y = 1.5
max_x = 422
min_x = 12
# max angle deflection (in rad) 
y_rad_change=0.1
# PID controller
pixel_1m_y = (max_y-min_y)/0.2
pixel_1m_x = (max_x-min_x)/0.13
print(f"pix_y:  {pixel_1m_y}" )
print(f"pix_x:  {pixel_1m_x}" )
# The axis may need to be flip / check cam orientation
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
        cv2.circle(frame,center,radius,(0,255,0),2)
        print("radius:",radius)
        r=np.append(r, radius)
        cv2.circle(frame,center,radius=5, color=(0, 0, 255), thickness=-1)
        
        print(time.time()-t0)
        np.append(t,time.time()-t0)
        print(f"x:{x},y:{y}")
        print(f"t:{t}")

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(r)
        
        sns.displot(r, kde=True, discrete=True)
        plt.show()
        break
cap.release()
cv2.destroyAllWindows()
time_now = time.time()
timediff = time_now - t0
print(f"t0 = {t0}")
print(f"time now = {time_now}")
print(f"Time diff = {timediff}")


