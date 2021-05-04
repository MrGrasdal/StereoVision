//
// Created by martin on 28.04.2021.
//

#include "camModel.h"
#include "Eigen/Dense"

CameraModel::CameraModel(bool right, double _fx, double _fy, double _cx, double _cy) {
    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;
    d1 = d2 = d3 = 0;

    if (right == false)
    {
        trans = Eigen::Vector3f{0.0, 0.0, 0.0};
        rEuler = Eigen::Vector3f{0.0, 0.0, 0.0};
    }
    else
    {
        trans = Eigen::Vector3f{0.0, 1.5, 0.0};
        rEuler = Eigen::Vector3f{0.0, 0.0, 2.0};
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

void CameraModel::updatePosition(Eigen::Vector3f newTrans, Eigen::Vector3f newAngle) {
    trans = newTrans;
    rEuler = newAngle;
    R = eulerToRotMat(rEuler);

}



Eigen::Matrix3f CameraModel::calcK_matrix()
{
    Eigen::Matrix3f K_;
    K_ <<   fx, 0,  cx,
            0,  fy, cy,
            0,  0,  1;

    return K_;
}

Matrixf34 CameraModel::calcP_matrix()
{
    Matrixf34 P_;
    P_ <<   1, 0, 0, -trans[0],
            0, 1, 0, -trans[1],
            0, 0, 1, -trans[2];


    return K * R * P_;
}


Eigen::Matrix4f triTensor(Matrixf34 Pa, Matrixf34 Pb, Matrixf34 Pc, int q, int r, int l)
{
    Eigen::Matrix4f Tmat;
    Tmat << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;

    int k = 0;
    for (int i = 0; i < 3; i++) {
        if ( i != l-1) {
            Tmat.row(i) = Pa.row(k);
            k++;
        }
    }
    Tmat.row(2) = Pb.row(q);
    Tmat.row(3) = Pb.row(r);

    double T = Tmat.determinant();

    T = pow(-1, l+1) * T;


    std::cout << T << std::endl;

    return Tmat;
}


Eigen::Matrix3f CameraModel::eulerToRotMat(Eigen::Vector3f &theta)
{
    double roll = deg2Rad(theta[1]);
    double pitch = deg2Rad(theta[1]);
    double yaw = deg2Rad(theta[2]);

    // Calculate rotation about x axis
    Eigen::Matrix3f R_x;
    R_x <<  1, 0,                0,
            0, cos(pitch), -sin(pitch),
            0, sin(pitch), cos(pitch);

    // Calculate rotation about y axis
    Eigen::Matrix3f R_y;
    R_y <<  cos(yaw),  0,  sin(pitch),
            0,           1,  0,
            -sin(yaw), 0,  cos(yaw);

    // Calculate rotation about z axis
    Eigen::Matrix3f R_z;
    R_z <<  cos(roll),  -sin(roll),   0,
            sin(roll),  cos(roll),    0,
            0,         0,           1;

    // Combined rotation matrix
    return R_z * R_x * R_y;  // Pass på rekkefølge
}

double CameraModel::deg2Rad(double degree)
{
    double pi = 3.14159265359;
    return (degree * (pi / 180));
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



cv::Point3d CameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect, const Matrixf34& P)
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



Eigen::Matrix3f CameraModel::calcFundamental(CameraModel left, CameraModel right) {

    Eigen::Matrix3f F;
    F = left.K.inverse().transpose() * left.R * Skew(right.trans)
            * right.R.transpose() * right.K.inverse();
    return F;
}

Eigen::Matrix3f CameraModel::Skew(Eigen::Vector3f v)
{
    Eigen::Matrix3f skew;
    skew << 0,      -v[2],  v[1],
            v[2],   0,      -v[0],
            -v[1],  v[0],   0;
    return skew;
}

void CameraModel::unpackPosData(calibration::gnssGGA::ConstPtr gnss,
                                calibration::orientation::ConstPtr orient)
{
    GNSSpos[0] = gnss->latitude;
    GNSSpos[1] = gnss->longitude;
    GNSSpos[2] = gnss->altitude;

    IMUrot[0] = orient->roll;
    IMUrot[1] = orient->pitch;
    IMUrot[2] = orient->yaw;

}
