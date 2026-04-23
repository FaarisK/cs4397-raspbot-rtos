import cv2
import numpy as np
import time
import os
import sys,PID
sys.path.append('/home/pi/project_demo/lib')
from McLumk_Wheel_Sports import *

# 加载YOLO模型
def load_yolo():
    classes = []
    with open("./yolov3/coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    
    # 使用OpenCV的DNN模块加载模型
    net = cv2.dnn.readNetFromDarknet("./yolov3/yolov3-tiny.cfg", "./yolov3/yolov3-tiny.weights")
    
    # 设置后端为OpenCV（修复权重加载问题）
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    
    # 获取输出层
    ln = net.getLayerNames()
    try:
        # OpenCV 3.4+ 使用不同方式获取输出层
        ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    except IndexError:
        # OpenCV 4.x 使用扁平列表
        ln = [ln[i - 1] for i in net.getUnconnectedOutLayers()]
    
    return net, classes, ln

# 使用YOLO检测目标
def detect_objects(img, net, output_layers, classes,target_class=None, confidence_threshold=0.5):
    (H, W) = img.shape[:2]
    
    # 创建blob - 使用更小的尺寸提高性能
    blob = cv2.dnn.blobFromImage(
        img, 
        1/255.0,  # 缩放因子
        (224, 224),  # 更小的输入尺寸提高速度
        swapRB=True, 
        crop=False
    )
    
    net.setInput(blob)
    outputs = net.forward(output_layers)
    
    boxes = []
    confidences = []
    classIDs = []
    
    for output in outputs:
        for detection in output:
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            
            if confidence > confidence_threshold:
                # 缩放边界框到原始图像尺寸
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")
                
                # 计算边界框左上角坐标
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))
                
                # 更新列表
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)
    
    # 应用非极大值抑制
    indices = cv2.dnn.NMSBoxes(boxes, confidences, confidence_threshold, 0.3)
    
    detected_objects = []
    if len(indices) > 0:
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            label = classes[classIDs[i]]
            
            # 如果指定了目标类别，只返回该类别
            if target_class is None or label == target_class:
                detected_objects.append({
                    'box': (x, y, w, h),
                    'confidence': confidences[i],
                    'class': label
                })
    
    return detected_objects

# 计算两个边界框的IOU（交并比） Calculate the Intersection over Union (IOU) of two bounding boxes
def calculate_iou(box1, box2):
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2
    
    # 计算交集区域 Calculate the intersection area
    xi1 = max(x1, x2)
    yi1 = max(y1, y2)
    xi2 = min(x1 + w1, x2 + w2)
    yi2 = min(y1 + h1, y2 + h2)
    
    inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
    
    # 计算并集区域 Calculate the union region
    box1_area = w1 * h1
    box2_area = w2 * h2
    union_area = box1_area + box2_area - inter_area
    
    # 避免除以零 Avoid dividing by zero
    iou = inter_area / union_area if union_area > 0 else 0
    return iou


def servo_reset():
    bot.Ctrl_Servo(1,90)
    bot.Ctrl_Servo(2,40)


def main():
    servo_reset()
    # init YOLO
    net, classes, output_layers = load_yolo()
    target_class = None
    target_servox = 90
    target_servoy = 25
    
    
    #舵机pid X轴
    Px_Servo = 0.6
    Ix_Servo = 0.2
    Dx_Servo = 0.01
    X_Middle_error = 160 #图像X轴中心  Image X-axis center
    X_track_PID = PID.PositionalPID(Px_Servo, Ix_Servo, Dx_Servo)
     
    #舵机pid y轴
    Py_Servo = 0.8 
    Iy_Servo = 0.2
    Dy_Servo = 0.01
    Y_Middle_error = 120 #图像y轴中心  Image X-axis center
    Y_track_PID = PID.PositionalPID(Py_Servo, Iy_Servo, Dy_Servo) 

    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    imshow_num = 0
    
    if not cap.isOpened():
        print("open camera fail!")
        return
    
    ret, frame = cap.read()
    if not ret:
        print("error")
        return
    
    # 选择初始追踪区域 Select initial tracking area
    bbox = (172,102,47,80)
    
    
    # 如果用户取消了选择，则退出 If the user cancels the selection, exit
    if bbox == (0, 0, 0, 0):
        print("BOX Error!!!")
        cap.release()
        return

    
    # 初始化KCF追踪器 Initialize KCF tracker
    param = cv2.TrackerKCF.Params()
    param.detect_thresh = 0.2
    tracker = cv2.TrackerKCF_create(param)
    tracker.init(frame, bbox)
    
    
    lost_count = 0
    max_lost_frames = 15  # 树莓派上允许更多的丢失帧数 Allow more frame loss on Raspberry Pi
    tracking_state = "TRACKING"  # TRACKING, LOST, REACQUIRED
    last_valid_bbox = bbox
    #detection_fps = 0
    
    fps_counter = 0
    start_time = time.time()
    fps = 0
    

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 更新追踪器 Update tracker
        #tracker_start = time.time()
        success, bbox = tracker.update(frame)
        # tracker_time = time.time() - tracker_start
        
        if success:
            lost_count = 0
            tracking_state = "TRACKING"
            
            # 绘制追踪框 Draw a tracking box
            x, y, w, h = [int(i) for i in bbox]
            
            #这里调节pid控制舵机运动，因为无法判断物体大小
            X_track_PID.SystemOutput = x + w/2
            X_track_PID.SetStepSignal(X_Middle_error)
            X_track_PID.SetInertiaTime(0.01, 0.1)
            #print(X_track_PID.SystemOutput)
            target_valuex = int(1500+X_track_PID.SystemOutput)
            target_servox = int((target_valuex-500)/10)
            # 将云台转动至PID调校位置 Turn the gimbal to the PID adjustment position
            if target_servox > 180:
                target_servox = 180
            if target_servox < 0:
                target_servox = 0     
            # 输入Y轴方向参数PID控制输入 Input Y-axis direction parameter PID control input
            Y_track_PID.SystemOutput = y + h/2
            Y_track_PID.SetStepSignal(Y_Middle_error)
            Y_track_PID.SetInertiaTime(0.01, 0.1)
            target_valuey = int(850+Y_track_PID.SystemOutput)
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
            
            
            last_valid_bbox = (x, y, w, h)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # 显示追踪状态 Display tracking status
            cv2.putText(frame, f"Tracking", (20, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 更新目标位置信息 Update target location information
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(frame, (center_x, center_y), 3, (0, 255, 0), -1)
            
        else:
            # stop_robot()#停止小车
            lost_count += 1
            tracking_state = f"LOST: {lost_count}/{max_lost_frames}"
            cv2.putText(frame, tracking_state, (20, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 在丢失位置绘制红色框 Draw a red box at the lost location
            x, y, w, h = [int(i) for i in last_valid_bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
            # 触发重检测机制 Trigger the re detection mechanism
            if lost_count > max_lost_frames // 3:  # 更早触发重检测 Trigger re detection earlier
                # 在最后已知位置附近创建搜索区域 Create a search area near the last known location
                search_margin = min(100, 50 + lost_count * 5)  # 动态调整搜索范围 Dynamically adjust search scope
                search_x1 = max(0, x - search_margin)
                search_y1 = max(0, y - search_margin)
                search_x2 = min(frame.shape[1], x + w + search_margin)
                search_y2 = min(frame.shape[0], y + h + search_margin)
                
                # 确保搜索区域有效 Ensure that the search area is valid
                if search_x2 > search_x1 and search_y2 > search_y1:
                    # 提取搜索区域 Extract search area
                    search_area = frame[search_y1:search_y2, search_x1:search_x2]
                    
                    # 绘制搜索区域 Draw search area
                    cv2.rectangle(frame, (search_x1, search_y1), 
                                 (search_x2, search_y2), (255, 255, 0), 2)
                    cv2.putText(frame, "Search Area", (search_x1, search_y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
                    # 每5帧进行一次重检测（平衡性能） Perform a re detection every 5 frames (balancing performance)
                    if lost_count % 5 == 0 and search_area.size > 0:
                        # 使用YOLO在搜索区域检测目标 Using YOLO to detect targets in the search area
                        # det_start = time.time()
                        detected_objs = detect_objects(
                            search_area, net, output_layers, classes,
                            target_class=target_class,
                            confidence_threshold=0.4  # 降低置信度阈值
                        )
                        # detection_time = time.time() - det_start
                        # detection_fps = 1 / detection_time if detection_time > 0 else 0
                        
                        best_match = None
                        max_iou = 0.0
                        
                        for obj in detected_objs:
                            ox, oy, ow, oh = obj['box']
                            
                            # 转换到原始图像坐标 Convert to original image coordinates
                            abs_x = search_x1 + ox
                            abs_y = search_y1 + oy
                            abs_box = (abs_x, abs_y, ow, oh)
                            
                            iou = calculate_iou(last_valid_bbox, abs_box)
                            
                            # 选择最佳匹配 Choose the best match
                            if iou > max_iou:
                                max_iou = iou
                                best_match = abs_box
                        
                        # 如果找到匹配的目标，重新初始化追踪器 If a matching target is found, reinitialize the tracker
                        if best_match and max_iou > 0.2:  # 降低IOU阈值 Reduce IOU threshold
                            # 更新追踪器 Update tracker
                            tracker = cv2.TrackerKCF_create()
                            tracker.init(frame, best_match)
                            lost_count = 0
                            tracking_state = "REACQUIRED"
                            
                            # 绘制重新获取的框 Draw the reacquired box
                            x, y, w, h = [int(i) for i in best_match]
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
                            cv2.putText(frame, "Reacquired!", (x, y-10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        
        # FPS
        fps_counter += 1
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= 1.0:  
            fps = fps_counter / elapsed_time
            start_time = current_time
            fps_counter = 0
        
                
        imshow_num +=1
        if imshow_num%2==0:
            cv2.putText(frame, f"FPS: {fps:.1f}", (20, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
            cv2.putText(frame, f"State: {tracking_state}", (20, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.imshow("Track_Food", frame)
            imshow_num = 0
        
        
        
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27: 
            break
    
    cap.release()
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()