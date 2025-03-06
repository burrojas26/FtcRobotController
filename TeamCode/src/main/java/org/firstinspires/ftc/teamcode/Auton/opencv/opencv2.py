import cv2
import random
img = cv2.imread('assets/test_img.png', -1)
#print(img)
#OpenCV uses BGR, not RGB
#print(img[257][45:400])
for i in range(400):
    for g in range(img.shape[1]):
        img[i][g]=[random.randint(0,255), random.randint(0,255), random.randint(0,255)]
cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows(0)


