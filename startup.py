from McLumk_Wheel_Sports import *
import cv2

def StartupAction():
    #import Enum
    
    boundaries = [
    	([17, 15, 100], [50, 56, 200]),
    	([86, 31, 4], [220, 88, 50]),
    	([25, 146, 190], [62, 174, 250])
    ]
    
    rotate_left(100)
    time.sleep(1.4) #spin in one full rotation
    stop_robot()
    print("Hello there!\n")