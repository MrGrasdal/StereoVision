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



};


#endif //SRC_EKF_H
