import cv2
import numpy as np
img = cv2.imread('assets/template_test_2.jpeg', 0)
template = cv2.imread('assets/ftcAT1.png', 0)
h1, w1 = template.shape
h = np.int0(h1/3)
w = np.int0(w1/3)
#methods = [cv2.TM_CCOEFF, cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR, cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    location = max_loc
    bottom_right = (location[0] + w, location[1] + h)
    cv2.rectangle(frame, location, bottom_right, 255, 5)    
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()


