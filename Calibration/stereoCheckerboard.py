import numpy as np
import cv2 as cv
import glob
import os

pathLeft = 'imgs/mono_calib/right'
pathRight = 'imgs/mono_calib/left'

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.01)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((4 * 6, 3), np.float32)
objp[:, :2] = np.mgrid[0:6, 0:4].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objPts = []  # 3d point in real world space

imgPtsLeft = []  # 2d points in left image plane.
imgPtsRight = [] # 2d points in right image plane

imgList = sorted(os.listdir(pathLeft))

teller = 0
for fname in imgList:
    imgLeft = cv.imread(pathLeft + fname)
    imgRight = cv.imread(pathRight + fname)

    imgLeft = cv.cvtColor(imgLeft, cv.COLOR_BGR2GRAY)
    imgRight = cv.cvtColor(imgLeft, cv.COLOR_BGR2GRAY)

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    foundLeft, cornersLeft = cv.findChessboardCorners(imgLeft, (6, 4), None)
    foundRight, cornersRight = cv.findChessboardCorners(imgRight, (6, 4), None)

    # If found, add object points, image points (after refining them)
    if foundLeft:
        objPts.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgPtsLeft.append(corners2)

        # Draw and display the corners
        img = cv.drawChessboardCorners(img, (6, 4), corners, ret)

        # cv.imshow('img',img)
        # cv.waitKey(1000)
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objPts, imgPtsLeft, gray.shape[::-1], None, None)
print(mtx)

with open('camera_l_intr_original.txt', 'wb') as f:
    np.savetxt(f, mtx)

with open('camera_l_dist_original.txt', 'wb') as f:
    np.savetxt(f, dist)

path2 = 'files/test_calib_pics_l/1603384357.301529.bmp'

img_org = cv.imread(path2)
h, w = img_org.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

with open('camera_l_intr_optimized.txt', 'wb') as f:
    np.savetxt(f, newcameramtx)

print(roi)
print(newcameramtx)
cv.destroyAllWindows()