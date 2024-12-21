import cv2
img_test = cv2.imread('assets/test_img.png', -1)
# cv2.imwrite('new_img.png', test_img
cv2.imshow('image', img_test)
cv2.waitKey(0)
cv2.destroyAllWindows()
