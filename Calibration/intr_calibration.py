import numpy as np
import cv2
import glob
from tqdm import tqdm

path = 'imgs/calibRun/right/*.bmp'
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.01)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((4*6,3), np.float32)
objp[:,:2] = np.mgrid[0:600:100,0:400:100].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob(path)
teller = 0
i = 0

pbar = tqdm(total = len(images))

while i < len(images):

    fname = images[i]
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (6,4), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
 
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (6,4), corners, ret)
        teller += 1
        i += 20
        pbar.update(20)
        #cv2.imshow('img',img)
        #cv2.waitKey(1)

    else:
        i += 1
        pbar.update(1)

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
    if pVE[i,0] < 0.2:
        newObjPoints.append(objp)
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