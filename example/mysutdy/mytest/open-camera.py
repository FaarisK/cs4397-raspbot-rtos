import cv2

delt = 1

cap=cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
#cap.set(cv2.CAP_PROP_EXPOSURE, -156)
cap.set(3,320*delt)
cap.set(4,240*delt)


while True:
    ret, frame = cap.read()
    cv2.imshow("color_image", frame)
    cv2.waitKey(1)
