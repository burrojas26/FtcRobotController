import cv2
img = cv2.imread('assets/ftcAT6.png', -1)
#img_test = cv2.resize(img_test, (0,0), fx=0.5, fy=0.5)
#img_test = cv2.rotate(img_test, cv2.ROTATE_90_CLOCKWISE)
# cv2.imwrite('new_img.png', test_img
#cv2.imshow('image', img_test)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
scaleFactor=[0.8, 0.6, 0.4, 0.3, 0.2, 0.15]
for i in range(6):
    new_img = cv2.resize(img, (0,0), fx=scaleFactor[i], fy=scaleFactor[i])
    cv2.imwrite('assets/ftcAT6-'+str(i+1)+'.png', new_img)
