import cv2 as cv
import numpy as np
import scipy.io

#mat = scipy.io.loadmat('files/stereo_LEFT_BB_right_RIGHT_IR_PG_IR.mat')

#mat2 = scipy.io.loadmat('files/BB_right_PG.mat')

#print(mat)
#
#print(mat2)

#mat = np.array([[1,1], [2,2], [3,3], [4,4]],[[5,5], [6,6], [7,7], [8,8]])


mat = np.array([[1,1], [1,2], [1,3], [1, 4]])
mat2 = np.array([[1,5], [1,6], [1,7], [1, 8]])


mat3 = np.array([[2,1], [2,2], [2,3], [2, 4]])
mat4 = np.array([[2,5], [2,6], [2,7], [2, 8]])
list3 = [mat, mat2]
list2 =  [mat3, mat4]

list = [list3,list2]

vst = np.vstack(list)
vst2 = np.vstack(vst)
print(mat)


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