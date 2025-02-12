import cv2
import numpy as np
img = cv2.imread('assets/other_test_img.jpeg')
#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#corners = cv2.goodFeaturesToTrack(gray, 100, 0.1, 10) #src, # of corners, min quality, min euclidean distance
#corners = np.int0(corners)
#for corner in corners:
#    x, y = corner.ravel()
#    cv2.circle(img, (x, y), 15, (255, 0, 0), -1)
#
#
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(gray, 25, 0.2, 10)
    corners = np.int0(corners)
    for corner in corners:
        x, y = corner.ravel()
        cv2.circle(frame, (x, y), 15, (255, 0, 0), -1)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

 