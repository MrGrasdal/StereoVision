//
// Created by martin on 03.05.2021.
//

#ifndef SRC_EKF_H
#define SRC_EKF_H

#include "Eigen/Dense"
#include <opencv2/core.hpp>
#include "camera.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class EKF {
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


    // state vector
    Eigen::VectorXd z_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;


    Eigen::Vector3d baseline;

    Eigen::MatrixXd Pleft;
    Eigen::MatrixXd Pright;
    Eigen::MatrixXd PleftNxt;

    Eigen::MatrixXd Fundamental;

    Eigen::MatrixXd dPleft;
    Eigen::MatrixXd dPright;
    Eigen::MatrixXd dPleftNxt;





    /**
     * Constructor
     */
    EKF();

    /**
     * Destructor
     */
    virtual ~EKF();

    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
              Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

    void Init(Camera camRight);
    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k and k+1 in s
     */
    void Predict();

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd &z);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void UpdateEKF(const Eigen::VectorXd &z);

    void ProcessMeasurement(Camera nxtLeft, Camera left, Camera right);

    MatrixXd CalculateJacobian(const VectorXd& x_state);

private:
    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;


    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd Hj_;
    float noise_ax;
    float noise_ay;


    MatrixXd CalculateA(Camera leftNxt, Camera left, Camera right);
    MatrixXd CalculateB(Camera left, Camera right, Camera leftNxt);

    MatrixXd derivatePa(int param);
    MatrixXd derivatePb(int param);
    MatrixXd derivatePc(int param);

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


#endif //SRC_EKF_H
