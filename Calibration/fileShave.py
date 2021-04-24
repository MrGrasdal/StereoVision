import os
import cv2 as cv

path = "/media/martin/Samsung_T5/imgs/calibDay2/left/"
savePath = "/media/martin/Samsung_T5/imgs/calibDay2/left2/"

imList = sorted(os.listdir(path))

for imName in imList:

    image = cv.imread(path+imName)
    newName = imName[7:]
    newName = newName[:-4] + ".bmp"

    cv.imwrite(savePath+newName, image)


