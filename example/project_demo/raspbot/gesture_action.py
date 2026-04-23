#!/usr/bin/env python3
# encoding: utf-8
import math
import time
import cv2 as cv
import numpy as np
import mediapipe as mp


import threading

class handDetector:
    def __init__(self, mode=False, maxHands=1, detectorCon=0.5, trackCon=0.5):
        self.tipIds = [4, 8, 12, 16, 20]
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon
        )
        self.lmList = []
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

        self.last_action = ""
        self.repeat = 0
        self.count = 0

        task_1 = threading.Thread(target=self.task_control_timeout, name="task_control_timeout")
        task_1.setDaemon(True)
        task_1.start()

    def get_dist(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))

    def calc_angle(self, pt1, pt2, pt3):
        point1 = self.lmList[pt1][1], self.lmList[pt1][2]
        point2 = self.lmList[pt2][1], self.lmList[pt2][2]
        point3 = self.lmList[pt3][1], self.lmList[pt3][2]
        a = self.get_dist(point1, point2)
        b = self.get_dist(point2, point3)
        c = self.get_dist(point1, point3)
        try:
            radian = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
            angle = radian / math.pi * 180
        except:
            angle = 0
        return abs(angle)


    def findHands(self, frame, draw=True):
        self.lmList = []
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for i in range(len(self.results.multi_hand_landmarks)):
                if draw: self.mpDraw.draw_landmarks(frame, self.results.multi_hand_landmarks[i], self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                self.mpDraw.draw_landmarks(img, self.results.multi_hand_landmarks[i], self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                for id, lm in enumerate(self.results.multi_hand_landmarks[i].landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    self.lmList.append([id, cx, cy])
        return frame, img

    # 寻找某个指关节的坐标
    def findPoint(self, point):
        return self.lmList[point][1], self.lmList[point][2]
    
    def frame_combine(slef,frame, src):
        if len(frame.shape) == 3:
            frameH, frameW = frame.shape[:2]
            srcH, srcW = src.shape[:2]
            dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        else:
            src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            frameH, frameW = frame.shape[:2]
            imgH, imgW = src.shape[:2]
            dst = np.zeros((frameH, frameW + imgW), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        return dst
    
    # 统计手指数量
    def fingersUp(self):
        fingers=[]
        if (self.calc_angle(self.tipIds[0], self.tipIds[0] - 1, self.tipIds[0] - 2) > 150.0) and \
            (self.calc_angle(self.tipIds[0] - 1, self.tipIds[0] - 2, self.tipIds[0] - 3) > 150.0):
                fingers.append(1)
        else:
            fingers.append(0)
        # 4 finger
        for id in range(1, 5):
            if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers
    

    def get_gesture(self):
        gesture = ""
        fingers = self.fingersUp()
        if fingers.count(1) == 3: gesture = "Three"
        elif fingers.count(1) == 4: gesture = "Four"
        elif fingers.count(1) == 0: gesture = "Zero"
        elif fingers.count(1) == 1: gesture = "One"
        elif fingers.count(1) == 2:
            if fingers[0] == 1 and fingers[4] == 1: gesture = "Six"
            elif fingers[0] == 1 and self.calc_angle(4, 5, 8) > 90: gesture = "Eight"
            else: gesture = "Two"
        elif fingers.count(1)==5:
            if self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) < 60 and \
                    self.get_dist(self.lmList[4][1:], self.lmList[12][1:]) < 60 and \
                    self.get_dist(self.lmList[4][1:], self.lmList[16][1:]) < 60 and \
                    self.get_dist(self.lmList[4][1:], self.lmList[20][1:]) < 60 : gesture = "Seven"
            else:
                gesture = "Five"
        return gesture


    def get_gesture_control(self):
        gesture = ""
        fingers = self.fingersUp()
        if fingers.count(1) == 3: gesture = "Three"
        elif fingers.count(1) == 4: gesture = "Four"
        elif fingers.count(1) == 0: gesture = "Zero"
        elif fingers.count(1) == 1: gesture = "One"
        elif fingers.count(1) == 2:
            if fingers[1] == 1 and fingers[4]==1: gesture = "Rock"
            else: gesture = "Two"
        elif fingers.count(1)==5:
                gesture = "Five"
        return gesture
    
    
    def parse_action(self, action):
        if self.last_action == action:
            self.repeat = self.repeat + 1
            if self.repeat > 5:
                self.repeat = 0
                self.control_robot(action)
        else:
            self.repeat = 0
            self.last_action = action

    def robot_action(self, action):
        state = True
        # if action == "One":
            
        # elif action == "Two":
            
        # elif action == "Three":
            
        # elif action == "Four":
            
        # elif action == "Five":
            
        # elif action == "Six":
            
        # elif action == "Seven":
            
        # elif action == "Eight":
            
        # else:
        #     state = False
        return state
        
    
    def control_robot(self, action):
        if self.count > 0:
            return
        self.count = 20
        self.robot_action(action)
        # if action == "One":
        #     self.robot_action(1)
        # elif action == "Two":
        #     self.robot_action(2)
        # elif action == "Three":
        #     self.robot_action(3)
        # elif action == "Four":
        #     self.robot_action(4)
        # elif action == "Five":
        #     self.robot_action(5)
        # elif action == "Six":
        #     self.robot_action(6)
        # elif action == "Seven":
        #     self.robot_action(7)
        # elif action == "Eight":
        #     self.robot_action(8)
        # else:
        #     self.count = 0
        if self.count > 0:
            print("action:", action)

    def task_control_timeout(self):
        while True:
            time.sleep(.1)
            if self.count > 0:
                self.count = self.count - 1


'''
One Two Three Four Five Six Seven Eight
'''
if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    pTime = cTime = 0
    hand_detector = handDetector(detectorCon=0.75)
    while capture.isOpened():
        ret, frame = capture.read()
        # frame = cv.flip(frame, 1)
        frame, img = hand_detector.findHands(frame, draw=False)
        if len(hand_detector.lmList) != 0:
            totalFingers = hand_detector.get_gesture()
            cv.rectangle(frame, (0, 430), (230, 480), (0, 255, 0), cv.FILLED)
            cv.putText(frame, str(totalFingers), (10, 470), cv.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            print("Finger", totalFingers)
            hand_detector.parse_action(totalFingers)
        if cv.waitKey(1) & 0xFF == ord('q'): break
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        dist = hand_detector.frame_combine(frame, img)
        cv.imshow('dist', dist)
        # cv.imshow('frame', frame)
        # cv.imshow('img', img)
    capture.release()
    cv.destroyAllWindows()
