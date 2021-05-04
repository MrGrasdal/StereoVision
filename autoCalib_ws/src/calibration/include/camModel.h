//
// Created by martin on 28.04.2021.
//

#ifndef SRC_CAMMODEL_H
#define SRC_CAMMODEL_H

#include <string.h>
#include <iostream>
#include <vector>
#include "Eigen/Dense"

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <calibration/gnssGGA.h>
#include <calibration/orientation.h>

using namespace std;
using namespace cv;

typedef Eigen::Matrix<float, 3, 4> Matrixf34;

class CameraModel {
    public:

    double fx, fy, cx, cy;
    double d1, d2, d3;

    Eigen::Vector3f trans;
    Eigen::Vector3f rEuler;

    Eigen::Vector3f GNSSpos;
    Eigen::Vector3f IMUrot;

    Eigen::Matrix3f K;
    Eigen::Matrix3f R;
    Matrixf34 P;



    CameraModel(const CameraModel &sample);


    CameraModel(bool right=false, double _fx=1000.0, double _fy=1000.0,
                double _cx=600.0, double _cy=500.0);

    Eigen::Matrix3f calcK_matrix();

    Matrixf34 calcP_matrix();

    static Eigen::Matrix3f calcFundamental(CameraModel left, CameraModel right);

    static Eigen::Matrix3f triTensor(Matrixf34 Pa, Matrixf34 Pb, Matrixf34 Pc, int q, int r, int l);

    Point2d distortionModel(Point2d pt);

    Point3d projectPixelTo3dRay(const Point2d &uv_rect, const Matrixf34 &P);

    Point2d project3dToPixel(const Point3d &xyz);

    void unpackPosData(calibration::gnssGGA_<allocator<void>>::ConstPtr gnss,
                       calibration::orientation_<allocator<void>>::ConstPtr orient);

private:

    Eigen::Matrix3f eulerToRotMat(Eigen::Vector3f &theta);

    static Eigen::Matrix3f Skew(Eigen::Vector3f v);


    void updatePosition(Eigen::Vector3f newTrans, Eigen::Vector3f newAngle);


    double deg2Rad(double degree);
};




#endif //SRC_CAMMODEL_H
