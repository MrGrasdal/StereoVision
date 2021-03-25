import numpy as np
import cv2
import os
from tqdm import tqdm

from functions import *

path = 'imgs/aprilCrazy2/right/'
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.01)

atDet = Detector(families='tag16h5',
                    nthreads=1,
                    quad_decimate=5.0,
                    quad_sigma=0.4,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)

#objPts = np.array([[750,145,0],[895,145,0],[895,0,0],[750,0,0],[0,145,0],[145,145,0],[145,0,0],[0,0,0]],dtype= np.float32)
objPts = np.array([[0,750,0],[0,895,0],[145,895,0],[145,750,0],[0,0,0],[0,145,0],[145,145,0],[145,0,0]],dtype= np.float32)
objPts[:,:2] = objPts[:,:2].T.reshape(-1,2)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = sorted(os.listdir(path))
teller = 0
i = 0

pbar = tqdm(total = len(images))

while i < len(images):
    fname = images[i]
    img = cv2.imread(path+fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgOk, tags = detectDoubletags(gray, atDet)
    # If found, add object points, image points (after refining them)
    if imgOk == True:
        objpoints.append(objPts)
        imgpoints.append(extractCorners(tags))
 
        # Draw and display the corners


        teller += 1
        i += 5
        pbar.update(5)
        #showAprilImage(img, tags)
        #cv2.imshow('img',img)
        #cv2.waitKey(1)

    else:
        i += 1
        pbar.update(1)

#imgpoints = np.vstack(imgpoints)

pbar.close()

print("No of images used: ", teller)

ret, mtx, dist, rvecs, tvecs, stdDevInt, stdDevExt, pVE = cv2.calibrateCameraExtended(objpoints, imgpoints, gray.shape[::-1],None,None)

save = False

if save:
    with open('files/K_l2.txt', 'wb') as f:
        np.savetxt(f, mtx)

    with open('files/D_l2.txt','wb') as f:
        np.savetxt(f,dist)

print("\nstdDevInt: \n", stdDevInt)
#print("\nstdDevExt: \n", stdDevExt)
print("\nperViewError: \n", pVE)
print("\n", mtx)


newObjPoints = []
newImgPoints = []

for i in range(len(imgpoints)):
    if pVE[i,0] < 100:
        newObjPoints.append(objPts)
        newImgPoints.append(imgpoints[i])


ret, mtx, dist, rvecs, tvecs, stdDevInt, stdDevExt, pVE = cv2.calibrateCameraExtended(newObjPoints, newImgPoints, gray.shape[::-1],mtx, dist)
print("Second run:\n", mtx)
print("\nstdDevInt: \n", stdDevInt)
#print("\nstdDevExt: \n", stdDevExt)
print("\nperViewError: \n", pVE)


#path2 = 'files/test_calib_pics_l/1603384357.301529.bmp'

#img_org = cv2.imread(path2)
#h,  w = img_org.shape[:2]
#newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

#with open('camera_l_intr_optimized.txt', 'wb') as f:
#    np.savetxt(f, newcameramtx)


#print(roi)
#print(newcameramtx)
cv2.destroyAllWindows()
