import numpy as np
import cv2 as cv
import glob
import os
from tqdm import tqdm

pathLeft = 'imgs/mono_calib/right/'
pathRight = 'imgs/mono_calib/left/'

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((4 * 6, 3), np.float32)
objp[:, :2] = np.mgrid[0:600:100, 0:400:100].T.reshape(-1, 2)
#objp[:, :2] = np.mgrid[0:6, 0:4].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objPts = []  # 3d point in real world space

imgPtsLeft = []  # 2d points in left image plane.
imgPtsRight = [] # 2d points in right image plane

imgList = sorted(os.listdir(pathLeft))

teller = 0
for k, fname in tqdm(enumerate(imgList)):
    imgLeft = cv.imread(pathLeft + fname, cv.IMREAD_GRAYSCALE)
    imgRight = cv.imread(pathRight + fname, cv.IMREAD_GRAYSCALE)

    #cv.imshow("", imgLeft)
    #k = cv.waitKey(0)
    #cv.destroyAllWindows()

    #grayLeft = cv.cvtColor(imgLeft, cv.COLOR_BGR2GRAY)
    #grayRight = cv.cvtColor(imgLeft, cv.COLOR_BGR2GRAY)

    foundLeft, cornersLeft = cv.findChessboardCorners(imgLeft, (6, 4), None)
    foundRight, cornersRight = cv.findChessboardCorners(imgRight, (6, 4), None)

    #img = cv.drawChessboardCorners(imgLeft, (6, 4), cornersLeft, foundLeft)
    #cv.imshow(str(k), img)
    #k = cv.waitKey(0)
    #cv.destroyAllWindows()

    # If found, add object points, image points (after refining them)
    if foundLeft and foundRight:
        cornersLeft = cv.cornerSubPix(imgLeft, cornersLeft, (11, 11), (-1, -1), criteria)
        cornersRight = cv.cornerSubPix(imgRight, cornersRight, (11, 11), (-1, -1), criteria)
        imgPtsLeft.append(cornersLeft)
        imgPtsRight.append(cornersRight)
        objPts.append(objp)

        # Draw and display the corners
        #img = cv.drawChessboardCorners(imgLeft, (6, 4), cornersLeft, foundLeft)
        #cv.imshow(str(k ), img)
        #k = cv.waitKey(0)
        #cv.destroyAllWindows()


KLeft = np.loadtxt('files/K_l.txt')
KRight = np.loadtxt('files/K_r.txt')

DLeft = np.loadtxt('files/D_l.txt')
DRight = np.loadtxt('files/D_r.txt')

retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = \
    cv.stereoCalibrate(objPts, imgPtsLeft, imgPtsRight, KLeft, DLeft, KRight, DRight, (1224,1024),criteria)

print(KLeft)
print(T)
print(R)





'''
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
'''