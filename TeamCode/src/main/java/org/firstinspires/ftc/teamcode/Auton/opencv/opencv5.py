import cv2
import numpy as np
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    width = int(cap.get(3))
    height = int(cap.get(4))
    hsv = cv2.cvtColor(frame, cv2.color_BGR2HSV)
    lower_blue = np.array([cv2.cvtColor([[[255, 0, 0]]])]) #PICK UP HERE
    upper_blue = np.array([])
    cv2.imshow('frame', img)
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

