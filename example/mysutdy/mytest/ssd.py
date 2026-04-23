import cv2
print(cv2.__version__)  # 输出版本（应显示 4.x 或 3.x）

# 尝试创建跟踪器
try:
    tracker = cv2.TrackerKCF_create()
    print("TrackerKCF_create() 成功！")
except:
    try:
        tracker = cv2.Tracker_create("KCF")
        print("Tracker_create('KCF') 成功！")
    except:
        print("失败：请检查 opencv-contrib-python 是否安装")