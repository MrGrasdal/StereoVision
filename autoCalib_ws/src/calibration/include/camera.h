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

        vector<KeyPoint> allKpts;
        vector<KeyPoint> currKpts;
        vector<KeyPoint> epochKpts;
        Mat allDesc;
        Mat currDesc;
        Mat epochDesc;

        CameraModel model;

        double time;

        Camera() {}

        Camera(double time, bool right=false);

        Camera(const Camera& sample);

        void initCamera(bool right=false);

};


#endif //SRC_CAMERA_H
