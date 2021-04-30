//
// Created by martin on 30.04.2021.
//

#ifndef SRC_CAMERA_H
#define SRC_CAMERA_H

#include <string.h>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "camModel.h"

using namespace std;
using namespace cv;

class Camera {
    public:
        Mat img;

        vector<KeyPoint> kpts;
        Mat desc;

        CameraModel model;

        double time;

        Camera(double time, bool right=false);

        Camera(const Camera& sample);

};


#endif //SRC_CAMERA_H
