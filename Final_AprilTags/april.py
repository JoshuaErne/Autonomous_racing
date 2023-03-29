# import numpy as np
# import cv2 as cv
# import matplotlib.pyplot as plt
# import time

# t0 = time.time()
# img1 = cv.imread('f12crop.png',cv.IMREAD_GRAYSCALE)          # queryImage

# img2 = cv.imread('f13crop-cutout.png',cv.IMREAD_GRAYSCALE)  # trainImage

# scale_percent = 220 # percent of original size
# width = int(img2.shape[1] * scale_percent / 100)
# height = int(img2.shape[0] * scale_percent / 100)
# dim = (width, height)
  
# # resize image
# resized = cv.resize(img2, dim, interpolation = cv.INTER_AREA)

# # Initiate SIFT detector
# sift = cv.SIFT_create()
# #orb = cv.ORB_create()

# # find the keypoints and descriptors with SIFT
# kp1, des1 = sift.detectAndCompute(img1,None)
# kp2, des2 = sift.detectAndCompute(resized,None)
# #kp1, des1 = orb.detectAndCompute(img1,None)
# #kp2, des2 = orb.detectAndCompute(resized,None)
# # BFMatcher with default params
# bf = cv.BFMatcher()
# matches = bf.knnMatch(des1,des2,k=2)
# # Apply ratio test
# good = []
# for m,n in matches:
#     if m.distance < 0.55*n.distance:
#         good.append([m])
# # cv.drawMatchesKnn expects list of lists as matches.
# img3 = cv.drawMatches(img1,kp1,resized,kp2,good,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

# t1 = time.time()
# total = t1-t0
# print('time taken', total)

# plt.imshow(img3),plt.show()


import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import time

t0 = time.time()
img1 = cv.imread('f12crop.png',cv.IMREAD_GRAYSCALE)          # queryImage

img2 = cv.imread('f13crop-cutout.png',cv.IMREAD_GRAYSCALE)  # trainImage

# scale_percent = 220 # percent of original size
# width = int(img2.shape[1] * scale_percent / 100)
# height = int(img2.shape[0] * scale_percent / 100)
# dim = (width, height)
  
# # resize image
# resized = cv.resize(img2, dim, interpolation = cv.INTER_AREA)
# Initiate SIFT detector
sift = cv.SIFT_create()
# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)
# BFMatcher with default params
bf = cv.BFMatcher()
matches = bf.knnMatch(des1,des2,k=2)
# Apply ratio test
good = []
for m,n in matches:
    if m.distance < 0.55*n.distance:
        good.append([m])
# cv.drawMatchesKnn expects list of lists as matches.
img3 = cv.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

t1 = time.time()
total = t1-t0
print('time taken', total)

plt.imshow(img3),plt.show()
