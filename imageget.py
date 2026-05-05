import cv2
import numpy as np

def gethsv() :
    cam = cv2.VideoCapture(0)
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height  = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))

    while True:
        ret, frame = cam.read()
        if ret:
            # Convert the frame to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            pixel = frame[240, 320]
            pixel = np.uint8([[pixel]])
            hsv_pixel = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)
        
            print(f"HSV Value: {hsv_pixel[0][0]}")
        
        #out.write(frame)
        #cv2.imshow('Camera', frame)
            #stop when Q is pressed
        #if cv2.waitKey(1) == ord('q'):
        break
    cam.release()
    out.release()
    cv2.destroyAllWindows()
    return hsv

