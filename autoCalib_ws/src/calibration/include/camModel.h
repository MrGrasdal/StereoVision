//
// Created by martin on 28.04.2021.
//

#ifndef SRC_CAMMODEL_H
#define SRC_CAMMODEL_H

#include <string.h>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

class CameraModel {

    public:

    double fx, fy, cx, cy;
    double d1, d2, d3;

    Vec3f trans;
    Vec3f rEuler;

    Mat K;
    Mat R;
    Mat P;

    CameraModel(const CameraModel &sample);


    CameraModel(bool right=false, double _fx=1000.0, double _fy=1000.0,
                double _cx=600.0, double _cy=500.0);

    Mat calcK_matrix();

    Mat calcP_matrix();

    static Mat calcFundamental(CameraModel left, CameraModel right);

    static Mat triTensor(Mat Pa, Mat Pb, Mat Pc, int q, int r, int l);

    Point2d distortionModel(Point2d pt);

    Point3d projectPixelTo3dRay(const Point2d &uv_rect, const Matx34d &P);

    Point2d project3dToPixel(const Point3d &xyz);


private:

    Mat eulerToRotMat(Vec3f &theta);

    static Mat Skew(Vec3f v);


};




#endif //SRC_CAMMODEL_H
