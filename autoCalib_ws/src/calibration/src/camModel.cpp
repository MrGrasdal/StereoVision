//
// Created by martin on 28.04.2021.
//

#include "camModel.h"

CameraModel::CameraModel(bool right, double _fx, double _fy, double _cx, double _cy) {
    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;
    d1 = d2 = d3 = 0;

    if (right == false)
    {
        trans = Vec3f{0.0, 0.0, 0.0};
        rEuler = Vec3f{0.0, 0.0, 0.0};
    }
    else
    {
        trans = Vec3f{0.0, 1.5, 0.0};
        rEuler = Vec3f{0.0, 0.0, 2.0};
    }

    K = calcK_matrix();
    R = eulerToRotMat(rEuler);
    P = calcP_matrix();
}

CameraModel::CameraModel(const CameraModel& sample) {
    fx = sample.fx;
    fy = sample.fy;
    cx = sample.cx;
    cy = sample.cy;
    d1 = sample.d1;
    d2 = sample.d2;
    d2 = sample.d3;
    trans = sample.trans;
    rEuler = sample.rEuler;
    K = sample.K;
    R = sample.R;
    P = sample.P;
}



Mat CameraModel::calcK_matrix()
{
    Mat K_ = (Mat_<double>(3, 3) <<
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1);

    return K_;
}

Mat CameraModel::calcP_matrix()
{
    Mat P_ = K * R * (Mat_<double>(3, 4) <<
            1, 0, 0, -trans[0],
            0, 1, 0, -trans[1],
            0, 0, 1, -trans[2]);

    return P_;
}


Mat CameraModel::triTensor(Mat Pa, Mat Pb, Mat Pc, int q, int r, int l )
{
    Mat Tmat = (Mat_<double>(4, 4) <<
                                   0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0);

    int k = 0;
    for (int i = 0; i < 3; i++) {
        if ( i != l-1) {
            Pa.row(i).copyTo(Tmat.row(k));
            k++;
        }
    }

    Pb.row(q).copyTo(Tmat.row(2));
    Pc.row(r).copyTo(Tmat.row(3));

    double T = determinant(Tmat);

    T = pow(-1, l+1) * T;


    std::cout << T << std::endl;

    return Tmat;
}


Mat CameraModel::eulerToRotMat(Vec3f &theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
            1, 0,                0,
            0, cos(theta[0]), -sin(theta[0]),
            0, sin(theta[0]), cos(theta[0]));

    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
            cos(theta[1]),  0,  sin(theta[1]),
            0,                 1,  0,
            -sin(theta[1]), 0,  cos(theta[1]));

    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
            cos(theta[2]),  -sin(theta[2]),   0,
            sin(theta[2]),  cos(theta[2]),    0,
            0,                 0,                   1);

    // Combined rotation matrix
    return R_z * R_x * R_y;  // Pass på rekkefølge
}

Point2d CameraModel::distortionModel(Point2d pt)
{
    double r2 = pow(pt.x,2) + pow(pt.x,2);

    return pt * ( 1 + d1 * r2 + d2 * pow(r2, 2) + d3 * pow(r2,3));
}

Point2d CameraModel::project3dToPixel(const cv::Point3d& xyz)
{

    // [U V W]^T = P * [X Y Z 1]^T
    // u = U/W
    // v = V/W
    cv::Point2d uv_rect;
    uv_rect.x = (fx*xyz.x + trans[0]) / xyz.z + cx;
    uv_rect.y = (fy*xyz.y + trans[1]) / xyz.z + cy;
    return uv_rect;
}



cv::Point3d CameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect, const cv::Matx34d& P)
{

    const double& fx = P(0,0);
    const double& fy = P(1,1);
    const double& cx = P(0,2);
    const double& cy = P(1,2);
    const double& Tx = P(0,3);
    const double& Ty = P(1,3);

    cv::Point3d ray;
    ray.x = (uv_rect.x - cx - Tx) / fx;
    ray.y = (uv_rect.y - cy - Ty) / fy;
    ray.z = 1.0;
    return ray;
}



Mat CameraModel::calcFundamental(CameraModel left, CameraModel right) {

    Mat F = left.K.inv().t() * left.R * Skew(right.trans) * right.R.t() * right.K.inv();
    return F;
}

Mat CameraModel::Skew(Vec3f v)
{
    Mat skew = (Mat_<double>(3,3) <<
            0,      -v[2],  v[1],
            v[2],   0,      -v[0],
            -v[1],  v[0],   0);
    return skew;
}
