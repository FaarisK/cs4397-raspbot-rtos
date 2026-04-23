import cv2
import time

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE, -156)
# 获取视频的帧率
fps = cap.get(cv2.CAP_PROP_FPS)

# 初始化帧计数和开始时间
frame_count = 0
start_time = time.time()

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # 更新帧计数
    frame_count += 1

    # 计算并显示FPS
    elapsed_time = time.time() - start_time
    fps = frame_count / elapsed_time
    cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
