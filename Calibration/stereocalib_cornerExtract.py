import cv2 as cv
import numpy as np
import glob
from pupil_apriltags import Detector

path_l = 'imgs/stereo_calib/leftImgs/*.bmp' #/left/*.bmp'
path_r = 'imgs/stereo_calib/rightImgs/*.bmp' #/right/*.bmp'

images_l = glob.glob(path_l)
images_r = glob.glob(path_r)

#img_l = cv.imread(path_l,cv.IMREAD_GRAYSCALE)
img_r = cv.imread(path_r,cv.IMREAD_GRAYSCALE)


at_detector = Detector(families='tag16h5',
                    nthreads=1,
                    quad_decimate=5.0,
                    quad_sigma=0.4,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)



#tags_l = at_detector.detect(img_l, estimate_tag_pose=False, camera_params=([1.238287703369286874e+03,1.238266713563885332e+03,6.092030391357441204e+02,5.385436978288130376e+02]), tag_size=0.145)
#tags_r = at_detector.detect(img_r, estimate_tag_pose=False, camera_params=([1.238262403759165181e+03,1.238410711249996211e+03,6.254788832466762187e+02,5.385136376243525547e+02]), tag_size=0.145)

#color_img_l = cv.cvtColor(img_l, cv.COLOR_GRAY2RGB)
#color_img_r = cv.cvtColor(img_r, cv.COLOR_GRAY2RGB)

#tags_l = tags_l[0:2]
#tags_r = tags_r[0:2]

#corners = np.empty((4*4,2))
corners = []
images = glob.glob(path_r)
images = sorted(images)


teller = 1

for k,fname in enumerate(images):
    img = cv.imread(fname,cv.IMREAD_GRAYSCALE)
    unfilteredTags = at_detector.detect(img, estimate_tag_pose=False, camera_params=([1.236642411038463251e+03, 1.237052151007248995e+03, 6.176210991753625876e+02, 5.309468943133282437e+02]), tag_size=0.145)
    color_img = cv.cvtColor(img, cv.COLOR_GRAY2RGB)
    print(str(k+1)+" "+fname[:41])

    filtTags = []
    id0 = False
    id1 = False

    for tag in unfilteredTags:
        #print(tag.pose_t)
        corners.append(tag.corners)
        print("\ttagid: {0} \tdecision_margin: {1}".format(tag.tag_id,tag.decision_margin))


        if tag.tag_id in range(0,2) and tag.decision_margin > 20:

            filtTags.append(tag)
            if tag.tag_id == 0:
                if id0:
                    continue
                else:
                    id0 = True

            if tag.tag_id == 1:
                if id1:
                    continue
                else:
                    id1 = True

            for idx in range(len(tag.corners)):
                cv.line(
                    color_img,
                    tuple(tag.corners[idx - 1, :].astype(int)),
                    tuple(tag.corners[idx, :].astype(int)),
                    (0, 255, 0),3,
                )

            cv.putText(
                color_img,
                str(tag.tag_id),
                org=(
                    tag.corners[0, 0].astype(int) + 10,
                    tag.corners[0, 1].astype(int) + 10,
                ),
                fontFace=cv.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255),
            )

    if id0 and id1:
        cv.imshow(str(k+1), color_img)
        k = cv.waitKey(0)
        cv.destroyAllWindows()

    id0 = False
    id1 = False

corners2 = np.vstack(corners)

#np.savetxt("files/cornersLeftStereoCalib.txt",corners2)
#print(corners)