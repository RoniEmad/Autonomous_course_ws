#!/usr/bin/env python3
# import the required libraries:
import rospy
import numpy as np 
import cv2 #import openCV to python


# Save image in set directory 
img = cv2.imread("/home/ronidodo/autonomous_ws/src/milestone3/scripts/test.png")  #Read image
print(img)
cv2.imshow("img", img)
cv2.waitKey(0)
#cv2.destroyAllWindows()
grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #Convert RGB image to grayscale
cv2.imshow("grayimg", grayImage)
cv2.waitKey(0)
#cv2.destroyAllWindows()

scale_percent = 15 # percent of original size
width = int(grayImage.shape[1] * scale_percent / 100)
height = int(grayImage.shape[0] * scale_percent / 100)
dim = (width, height)
# resize image
resized = cv2.resize(grayImage, dim)
print('Resized Dimensions : ',resized.shape)
print(resized)
cv2.imshow("Resized image", resized)

ret, bw_img = cv2.threshold(resized,155,1,cv2.THRESH_BINARY_INV) #Convert grayscale image to binary
print(type(bw_img))
bw_img = bw_img.astype(np.uint8)
cv2.imshow("Window", bw_img)

h, w= bw_img.shape #get image dimenssions
print('height:', h)
print('width:', w)
print(bw_img)

cropped_image = bw_img[1:-1, 1:-1]
#cropped_image[47,24:33]=0
#cropped_image[48,24:29]=1
cv2.imshow("cropped", cropped_image)

h, w= cropped_image.shape #get image dimenssions
print('height:', h)
print('width:', w)
print(cropped_image)

ret2, duplicate = cv2.threshold(cropped_image,0,255,cv2.THRESH_BINARY) #Convert grayscale image to binary
cv2.imshow("255", duplicate)
kernel = np.ones((5,5), np.uint8)
img_erosion = cv2.erode(duplicate, kernel, iterations=1)
cv2.imshow('Erosion', img_erosion)
ret3, erosion01 = cv2.threshold(img_erosion,0,1,cv2.THRESH_BINARY) #Convert grayscale image to binary
cv2.imshow('Erosion01', erosion01)

cv2.waitKey(0) #Maintain output window until user presses a key 
cv2.imwrite("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_grid01.png",cropped_image)
cv2.imwrite("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_grid0255.png",duplicate)
cv2.imwrite("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_grideroded0255.png",img_erosion)
cv2.imwrite("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_grideroded01.png",erosion01)
cv2.destroyAllWindows() #Destroying present windows on screen

