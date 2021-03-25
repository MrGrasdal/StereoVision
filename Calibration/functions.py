import cv2 as cv
from pupil_apriltags import Detector
import numpy as np

def detectDoubletags(img, atDet):

    unfilteredTags = atDet.detect(img, estimate_tag_pose=False, camera_params=([1.236642411038463251e+03, 1.237052151007248995e+03, 6.176210991753625876e+02, 5.309468943133282437e+02]), tag_size=0.145)
    color_img = cv.cvtColor(img, cv.COLOR_GRAY2RGB)

    filtTags = []
    id0 = False
    id1 = False

    for tag in unfilteredTags:
        #print("\ttagid: {0} \tdecision_margin: {1}".format(tag.tag_id, tag.decision_margin))

        if tag.tag_id in range(0,2) and tag.decision_margin > 40:
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
                    str(idx+1),
                    org=(
                        int(tag.corners[idx, 0]),
                        int(tag.corners[idx, 1])
                    ),
                    fontFace=cv.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.5,
                    color=(0, 0, 255 ))


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

    ok = False


    if id0 and id1:
        #cv.imshow("lol", color_img)
        #k = cv.waitKey(0)
        cv.destroyAllWindows()
        ok = True

    return ok, filtTags

def showAprilImage(img, tags):

    for tag in tags:
        for idx in range(len(tag.corners)):
            cv.line(
                img,
                tuple(tag.corners[idx - 1, :].astype(int)),
                tuple(tag.corners[idx, :].astype(int)),
                (0, 255, 0), 3,
            )

            cv.putText(
                img,
                str(idx + 1),
                org=(
                    int(tag.corners[idx, 0]),
                    int(tag.corners[idx, 1])
                ),
                fontFace=cv.FONT_HERSHEY_SIMPLEX,
                fontScale=0.5,
                color=(0, 0, 255))

        cv.putText(
            img,
            str(tag.tag_id),
            org=(
                tag.corners[0, 0].astype(int) + 10,
                tag.corners[0, 1].astype(int) + 10,
            ),
            fontFace=cv.FONT_HERSHEY_SIMPLEX,
            fontScale=0.8,
            color=(0, 0, 255),)

    cv.imshow("lol", img)
    k = cv.waitKey()

def extractCorners(tags):
    corners = np.concatenate(((tags[0].corners).reshape(4,1,2), (tags[1].corners).reshape(4,1,2)), axis=0)


    return np.float32(corners)
