import os
import cv2 as cv

path = "imgs/reducedCalibRun/right/"
savePath = "imgs/redCalibRun/right/"

imList = sorted(os.listdir(path))

for imName in imList:

    image = cv.imread(path+imName)
    newName = imName[6:]
    newName = newName[:-4] + ".bmp"

    cv.imwrite(savePath+newName, image)


