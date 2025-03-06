import cv2
import time as tm
import numpy as np
t11 = cv2.imread('assets/ftcAT1-1.png', 0)
t12 = cv2.imread('assets/ftcAT1-2.png', 0)
t13 = cv2.imread('assets/ftcAT1-3.png', 0)
t14 = cv2.imread('assets/ftcAT1-4.png', 0)
t15 = cv2.imread('assets/ftcAT1-5.png', 0)
t16 = cv2.imread('assets/ftcAT1-6.png', 0)
t1 = [t11, t12, t13, t14, t15, t16]
def convTrack1():
    cap = cv2.VideoCapture(0)
    fMax = []
    fLoc = []
    fH = []
    fW = []
    fframe = []
    tm.sleep(1)
    for i in range(5):
        ret, frame = cap.read()
        fframe.append(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
        fMax.append(max(max_valA))
        fLoc.append(location)
        fH.append(h)
        fW.append(w)
    fMax_index = fMax.index(max(fMax))
    print("Final Max: " + str(max(fMax)))
    f_loc = fLoc[fMax_index]
    f_h = fH[fMax_index]
    f_w = fW[fMax_index]
    bottom_right = (f_loc[0] + f_w, f_loc[1] + f_h)
    cv2.rectangle(frame, f_loc, bottom_right, 255, 5)    
    cv2.imshow('frame', fframe[fMax_index])
convTrack1()


