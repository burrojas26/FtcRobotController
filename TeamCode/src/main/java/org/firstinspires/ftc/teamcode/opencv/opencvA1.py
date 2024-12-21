import cv2
import numpy as np
#BELOW are readings of differently sized images for tag ONE
t11 = cv2.imread('assets/ftcAT1-1.png', 0)
t12 = cv2.imread('assets/ftcAT1-2.png', 0)
t13 = cv2.imread('assets/ftcAT1-3.png', 0)
t14 = cv2.imread('assets/ftcAT1-4.png', 0)
t15 = cv2.imread('assets/ftcAT1-5.png', 0)
t16 = cv2.imread('assets/ftcAT1-6.png', 0)
t1 = [t11, t12, t13, t14, t15, t16]
#methods = [cv2.TM_CCOEFF, cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR, cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #START OF RESIZE ACCOUNT METHOD
    #essentially, this just takes the accuracy values for 6 sizes of the tag and picks the highest one
    #it's really laggy (until i find a better way to do it)
    #also, it displays incorrectly
    max_valA = []
    max_locA = []
    hA = []
    wA = []
    a=0
    for i in range(6):
            temp_img = t1[i]
            result = cv2.matchTemplate(gray, temp_img, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            print(cv2.minMaxLoc(result))
            max_valA.append(max_val)
            max_locA.append(max_loc)
            temph, tempw = temp_img.shape
            hA.append(temph)
            wA.append(tempw)
            a+=1
    max_val_index = max_valA.index(max(max_valA))
    location = max_locA[max_val_index]
    h = hA[max_val_index]
    w = wA[max_val_index]
    #END OF RESIZE ACCOUNT METHOD
    bottom_right = (location[0] + w, location[1] + h)
    #drawing the rectangle using h, w, bottom_right values
    cv2.rectangle(frame, location, bottom_right, 255, 5)    
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()


