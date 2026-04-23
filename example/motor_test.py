from McLumk_Wheel_Sports import *
import cv2
#import Enum

boundaries = [
	([17, 15, 100], [50, 56, 200]),
	([86, 31, 4], [220, 88, 50]),
	([25, 146, 190], [62, 174, 250])
]

rotate_left(100)
time.sleep(1.8) #spin in one full rotation
stop_robot()
print("hello")

cam = cv2.VideoCapture(0)
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height  = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))

while True:
    ret, frame = cam.read()
    out.write(frame)
    cv2.imshow('Camera', frame)
    #stop when Q is pressed
    if cv2.waitKey(1) == ord('q'):
        break
cam.release()
out.release()
cv2.destroyAllWindows()

for (lower, upper) in boundaries:
	lower = np.array(lower, dtype = "uint8")
	upper = np.array(upper, dtype = "uint8")
