import cv2 as cv
import numpy as np
import scipy.io

mat = scipy.io.loadmat('files/stereo_LEFT_BB_right_RIGHT_IR_PG_IR.mat')

mat2 = scipy.io.loadmat('files/BB_right_PG.mat')

print(mat)

print(mat2)


'''
array1 = np.loadtxt("files/cornersLeftStereoCalib.txt")


array2 = array1[:8,:].reshape((8,1,2))
array3 = array1[8:16,:].reshape((8,1,2))

#print(array2)
#print(array3)

array4 = []

array4.append(array2)
array4.append(array3)

print(array4)
'''