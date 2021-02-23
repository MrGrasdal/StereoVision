import cv2 as cv
import numpy as np
import glob
from pupil_apriltags import Detector
import os

from functions import *

path_l = 'imgs/stereo_calib/leftImgs/' # *.bmp' #/left/*.bmp'
path_r = 'imgs/stereo_calib/rightImgs/' # *.bmp' #/right/*.bmp'

savepath_l = 'imgs/stereo_calib/sorted/left/'
savepath_r = 'imgs/stereo_calib/sorted/right/'

atDet = Detector(families='tag16h5',
                    nthreads=1,
                    quad_decimate=5.0,
                    quad_sigma=0.4,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)

cornersLeft = []
cornersRight = []

imlist = sorted(os.listdir(path_l))

for k,fname in enumerate(imlist):
    imgLeft = cv.imread(path_l+fname, cv.IMREAD_GRAYSCALE)
    imgRight = cv.imread(path_r+fname, cv.IMREAD_GRAYSCALE)

    imgOkLeft, tagsLeft = detectDoubletags(imgLeft, atDet)
    imgOkRight, tagsRight = detectDoubletags(imgRight, atDet)

    if imgOkLeft and imgOkRight:
        cv.imwrite(savepath_l+fname, imgLeft)
        cv.imwrite(savepath_r+fname, imgRight)

        cornersLeft.append([t.corners for t in tagsLeft])
        cornersRight.append([t.corners for t in tagsRight])


cornersLeft = np.vstack(np.vstack(cornersLeft))
cornersRight = np.vstack(np.vstack(cornersRight))

imageR = cv.imread(path_r + imlist[0], cv.IMREAD_GRAYSCALE)

color_img = cv.cvtColor(imageR, cv.COLOR_GRAY2RGB)

np.savetxt("files/cornersLeft.txt", cornersLeft)
np.savetxt("files/cornersRight.txt", cornersRight)

