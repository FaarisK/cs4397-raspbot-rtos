import sys
#导入Raspbot驱动库 Import the Raspbot library
from Raspbot_Lib import Raspbot
# 创建Rosmaster对象 bot Create the Rosmaster object bot
bot = Raspbot()

import cv2
import mediapipe as mp
import ipywidgets.widgets as widgets
import threading
import time
import sys
import math

import PID
xservo_pid = PID.PositionalPID(0.6, 0.2, 0.01)#1.1 0.4 0.01
yservo_pid = PID.PositionalPID(0.8, 0.2, 0.01)

# 定义 target_servox 和 target_servoy 在外部 Define target_servox and target_servoy externally
target_servox = 90
target_servoy = 25
def servo_reset():
    bot.Ctrl_Servo(1,90)
    bot.Ctrl_Servo(2,40)
servo_reset()



class FaceDetector:
    def __init__(self, minDetectionCon=0.5):
        self.mpFaceDetection = mp.solutions.face_detection
        self.mpDraw = mp.solutions.drawing_utils
        self.facedetection = self.mpFaceDetection.FaceDetection(min_detection_confidence=minDetectionCon)

    def findFaces(self, frame):
        img_RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.facedetection.process(img_RGB)
        bboxs = []
        bbox=0,0,0,0
        if self.results.detections:
            for id, detection in enumerate(self.results.detections):
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, ic = frame.shape
                bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                       int(bboxC.width * iw), int(bboxC.height * ih)
                bboxs.append([id, bbox, detection.score])
                frame= self.fancyDraw(frame, bbox)
                # cv2.putText(frame, f'{int(detection.score[0] * 100)}%',
                #            (bbox[0], bbox[1] - 20), cv2.FONT_HERSHEY_PLAIN,
                #            3, (255, 0, 255), 2)
        return frame, bboxs, self.results.detections, bbox

    def fancyDraw(self, frame, bbox, l=30, t=5):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        cv2.rectangle(frame, (x, y),(x + w, y + h), (0,255,0), 2)
        # Top left x,y
        cv2.line(frame, (x, y), (x + l, y), (0,255,0), t)
        cv2.line(frame, (x, y), (x, y + l), (0,255,0), t)
        # Top right x1,y
        cv2.line(frame, (x1, y), (x1 - l, y), (0,255,0), t)
        cv2.line(frame, (x1, y), (x1, y + l), (0,255,0), t)
        # Bottom left x1,y1
        cv2.line(frame, (x, y1), (x + l, y1), (0,255,0), t)
        cv2.line(frame, (x, y1), (x, y1 - l), (0,255,0), t)
        # Bottom right x1,y1
        cv2.line(frame, (x1, y1), (x1 - l, y1), (0,255,0), t)
        cv2.line(frame, (x1, y1), (x1, y1 - l), (0,255,0), t)
        return frame


image = cv2.VideoCapture(0)
image_width = 320
image_height = 240
image.set(3, image_width)
image.set(4, image_height)


def Face_Recongnize():
    global x,w,y,h
    imshow_num = 0
    face_detector = FaceDetector(0.75)
    while image.isOpened():
        ret, frame = image.read()
       
        faces,_,descore,bbox= face_detector.findFaces(frame)
        x,y,w,h = bbox
        if descore:     
            #Proportion-Integration-Differentiation算法  PID algorithm
            # 输入X轴方向参数PID控制输入 Input X-axis direction parameter PID control input
            xservo_pid.SystemOutput = x + w/2
            xservo_pid.SetStepSignal(160)
            xservo_pid.SetInertiaTime(0.01, 0.1)
            #print(xservo_pid.SystemOutput)
            target_valuex = int(1500+xservo_pid.SystemOutput)
            target_servox = int((target_valuex-500)/10)
            # 将云台转动至PID调校位置 Turn the gimbal to the PID adjustment position
            if target_servox > 180:
                target_servox = 180
            if target_servox < 0:
                target_servox = 0     
            # 输入Y轴方向参数PID控制输入 Input Y-axis direction parameter PID control input
            yservo_pid.SystemOutput = y + h/2
            yservo_pid.SetStepSignal(int(image_height/2))
            yservo_pid.SetInertiaTime(0.01, 0.1)
            target_valuey = int(850+yservo_pid.SystemOutput)
            target_servoy = int((target_valuey-500)/10)                   
            #print("target_servoy %d", target_servoy)  
            if target_servoy > 110:
                target_servoy = 110
            if target_servoy < 0:
                target_servoy = 0          
            # 将云台转动至PID调校位置 Turn the gimbal to the PID adjustment position
            #robot.Servo_control(target_valuex,target_valuey)
                
            bot.Ctrl_Servo(1, target_servox)
            bot.Ctrl_Servo(2, target_servoy)

        
        imshow_num +=1
        if imshow_num%2==0:
            cv2.imshow("face", frame)
            imshow_num = 0
            
        if cv2.waitKey(1)==ord('q'):
            image.release()
            cv2.destroyAllWindows()
            break
       
       
Face_Recongnize()
