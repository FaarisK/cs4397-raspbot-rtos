# !/usr/bin/env python
# coding: utf-8
import random
import cv2 as cv
import numpy as np
import tkinter as tk


class update_hsv:
    def __init__(self):
        '''
        初始化一些参数
        '''
        self.image = None
        self.binary = None
        self.hsvname = None
        self.detected_colors = []  # 用于存储检测到的颜色名称
    def Image_Processing(self, hsv_range):
        '''
        形态学变换去出细小的干扰因素
        :param img: 输入初始图像
        :return: 检测的轮廓点集(坐标)
        '''
        (lowerb, upperb) = hsv_range
        # 复制原始图像,避免处理过程中干扰
        color_mask = self.image.copy()
        # 将图像转换为HSV。
        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素。
        color = cv.inRange(hsv_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色
        color_mask[color == 0] = [0, 0, 0]
        # 将图像转为灰度图
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
        # _, contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) #python2
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # python3
        return contours, binary

    def draw_contours(self, hsv_name, contours):
        '''
        采用多边形逼近的方法绘制轮廓
        '''
        self.hsvname=None
        for i, cnt in enumerate(contours):
            # 计算多边形的矩
            mm = cv.moments(cnt)
            if mm['m00'] == 0:
                continue
            cx = mm['m10'] / mm['m00']
            cy = mm['m01'] / mm['m00']
            # 获取多边形的中心
            (x, y) = (np.int_(cx), np.int_(cy))
            # 计算轮廓的?积
            area = cv.contourArea(cnt)
            # ?积?于10000
            if area > 800:
                # 绘制中?
                cv.circle(self.image, (x, y), 5, (0, 0, 255), -1)
                # 计算最小矩形区域
                rect = cv.minAreaRect(cnt)
                # 获取盒?顶点
                box = cv.boxPoints(rect)
                # 转成long类型
                box = np.int0(box)
                # 绘制最小矩形
                cv.drawContours(self.image, [box], 0, (255, 0, 0), 2)
                cv.putText(self.image, hsv_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                self.hsvname=hsv_name
                self.detected_colors.append(self.hsvname)
        return self.detected_colors



    def get_contours(self, img, color_hsv):
        binary = None
        self.hsvname=None
        # 规范输入图像大小
        self.image = cv.resize(img, (320, 240), )
        for key, value in color_hsv.items():
            # 检测轮廓点集
            color_contours, binary = self.Image_Processing(color_hsv[key])
            # 绘制检测图像,并控制跟随
            self.detected_colors=self.draw_contours(key, color_contours)
        colors= self.detected_colors.copy()
        self.detected_colors.clear()  
        return self.image, binary,colors
        

