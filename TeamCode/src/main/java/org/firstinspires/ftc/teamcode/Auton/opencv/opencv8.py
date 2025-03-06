import cv2
import numpy as np
cap = cv2.VideoCapture(0)
template = cv2.imread('assets/ftcAT1.png', 0)
w, h = template.shape[::-1]
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.5
    loc = np.where(res >= threshold)
    for pt in zip(*loc[::-1]):
        cv2.rectangle(frame, pt, (pt[0]+w, pt[1]+h), (255,0,0), 2)
    cv2.imshow('detected', frame)
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()