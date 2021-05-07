//
// Created by martin on 07.05.2021.
//

#ifndef SRC_EKF_UTILS_H
#define SRC_EKF_UTILS_H

#include "Eigen/Dense"
#include "camera.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class EKF_utils {

public:

    enum state
    {   epochTx, epochTy, epochTz, epochR, epochP, epochY,
        extR, extP, extY,
        fxL, fyL, cxL, cyL, d1L, d2L,
        fxR, fyR, cxR, cyR, d1R, d2R,
        last_state
    };

    enum dx
    {
        Px_l, Py_l, Px_r, Py_r, Px_nxt, Py_nxt, last_xd
    };

    Eigen::Vector3d baseline;

    Eigen::MatrixXd Pleft;
    Eigen::MatrixXd Pright;
    Eigen::MatrixXd PleftNxt;

    Eigen::MatrixXd Fundamental;

    Eigen::MatrixXd dPleft;
    Eigen::MatrixXd dPright;
    Eigen::MatrixXd dPleftNxt;



    MatrixXd CalculateA(Camera leftNxt, Camera left, Camera right);
    MatrixXd CalculateB(Camera left, Camera right, Camera leftNxt);

    MatrixXd derivatePa(int param, Eigen::VectorXd *state);
    MatrixXd derivatePb(int param, Eigen::VectorXd *state);
    MatrixXd derivatePc(int param, Eigen::VectorXd *state);

    double triTensor(Matrixf34 Pa, Matrixf34 Pb, Matrixf34 Pc, int l, int q, int r);
    double dT_lqr(Camera leftNxt, Camera left, Camera right, int l, int q, int r);

    Eigen::Matrix3d Skew(Eigen::Vector3f v);

    MatrixXd GetRotMatrix(double roll, double pitch, double yaw);

    MatrixXd derivateF(int param);

    void UpdateProjAndFundamental();
    void UpdateDerivProj(int param);


    double derivateGqr(Eigen::Vector3d leftKpt, Eigen::Vector3d rightKpt, Eigen::Vector3d nextKpt, int q, int r, int param);
    double derivateHe(Eigen::Vector3d leftKpt, Eigen::Vector3d rightKpt, int param);


    MatrixXd CalculateFpred(Camera left, Camera right, Camera leftNxt);
    MatrixXd CalculateBpred(Camera left, Camera right, Camera leftNxt);

    MatrixXd X_hat(Camera left, Camera right, Camera leftNxt);

    double CalculateGqr(Eigen::Vector3d leftKpt, Eigen::Vector3d rightKpt, Eigen::Vector3d nextKpt, int q, int r);

    MatrixXd Constraint(Camera left, Camera right, Camera leftNxt);

};


#endif //SRC_EKF_UTILS_H
