//
// Created by martin on 03.05.2021.
//

#include "EKF.h"
#include <iostream>

#define PI 3.14159265
#define stateSize 21


EKF::EKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    //Initalize the F, P & Q matrices.
    F_ = MatrixXd(4,4);
    P_ = MatrixXd(4,4);
    Q_ = MatrixXd(4,4);

    //Assign the values for the laser sensor matrix, H_laser
    H_laser_ << 1,0,0,0,
            0,1,0,0;

    //Set the values for acceleration noise
    noise_ax = 9;
    noise_ay = 9;
}

EKF::~EKF() {}

void EKF::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
               MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    z_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void EKF::Init(Camera camRight) {

    z_ = Eigen::VectorXd(stateSize);

    // Movement between epochs
    z_[epochTx] = 0; //x
    z_[epochTy] = 0; //y
    z_[epochTz] = 0; //z
    z_[epochR] = 0; //roll
    z_[epochP] = 0; //pitch
    z_[epochY] = 0; //yaw
    //Extrinsics
    z_[extR] = camRight.model.rEuler[0]; //roll
    z_[extP] = camRight.model.rEuler[1]; //pich
    z_[extY] = camRight.model.rEuler[2]; //yaw
    //Left Camera
    z_[fxL] = camRight.model.fx;
    z_[fyL] = camRight.model.fy;
    z_[cxL] = camRight.model.cx;
    z_[cyL] = camRight.model.cy;
    z_[d1L] = 0;    //Distortion
    z_[d2L] = 0;    //Distortion
    //Right Camera
    z_[fxR] = camRight.model.fx;
    z_[fyR] = camRight.model.fy;
    z_[cxR] = camRight.model.cx;
    z_[cyR] = camRight.model.cy;
    z_[d1R] = 0;    //Distortion
    z_[d2R] = 0;    //Distortion

    baseline = camRight.model.trans;

}

void EKF::ProcessMeasurement(Camera nxtLeft, Camera left, Camera right) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
/*
    if (!is_initialized_) {

        // first measurement
        cout << "EKF: \n";
        //cout << "Initializing state vector\n";
        z_ = VectorXd(4);
        //cout << "Setting intial state...\n";
        z_ << 1, 1, 0.5, 0.5; //remember to tweak the velocity values to adjust the RMSE in the beginning

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

            float rho = measurement_pack.raw_measurements_[0];
            float theta = measurement_pack.raw_measurements_[1];
            float rho_dot = measurement_pack.raw_measurements_[2];
            //cout << "Radar measurement - extracting coordinates\n";
            z_(0) = rho * cos(theta);
            z_(1) = rho * sin(theta);
            //cout << "State" << ekf_.z_;

        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

            //cout << "Laser measurement - extracting coordinates.\n";
            float px = measurement_pack.raw_measurements_[0];
            float py = measurement_pack.raw_measurements_[1];

            z_(0) = px;
            z_(1) = py;
            //cout << "Initial state is: \n";
            //cout << ekf_.z_ << "\n";
        }
        //Capture the timestamp
        previous_timestamp_ = measurement_pack.timestamp_;
        //assign initial values to the state transition matrix, F
        F_ <<   1,0,1,0,
                0,1,0,1,
                0,0,1,0,
                0,0,0,1;
        //assign initial values to the covariance matrix, P. Adjust the variance values to reflect uncertainty in initial state
        P_ <<   1,0,0,0,
                0,1,0,0,
                0,0,500,0,
                0,0,0,500;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        cout << "Completed initialization ofEKF.\n";
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
/*
    //Calculate deltaT
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    float dt2 = dt*dt;
    float dt3 = dt2*dt;
    float dt4 = dt3*dt;

    //Save the current timestamp for use in the next predict cycle
    previous_timestamp_ = measurement_pack.timestamp_;

    //Update the state transition matrix, F
    F_(0,2) = dt;
    F_(1,3) = dt;

    //Set the process covariance matrix, Q
    Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
            0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
            dt3/2*noise_ax, 0, dt2*noise_ax, 0,
            0, dt3/2*noise_ay, 0, dt2*noise_ay;
    //cout << "Process noise, Q is: \n";
    //cout << ekf_.Q_;
    Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/
/*
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        //Calculate the Jacobian matrix about the current predicted state and set the EKF state transition matrix, H
        //Tools Jacobian;
        Hj_ = tools.CalculateJacobian(ekf_.z_);
        H_ = Hj_;
        //cout << "Jacobian is:\n";
        //cout << Hj_;
        //Initialize the EKF object measurement covariance matrix, R, to the right size and assign the correct values
        R_ = MatrixXd(3,3);
        R_ = R_radar_;
        UpdateEKF(measurement_pack.raw_measurements_); //Comment this line to turn off radar updates

    } else {
        //Set the EKF object to use the LASER sensor matrix, H
        H_ = H_laser_;
        //Initialize the EKF object measurement covariance matrix, R, to the right size and assign the correct values
        R_ = MatrixXd(2,2);
        R_ = R_laser_;
        Update(measurement_pack.raw_measurements_); //Comment this line to turn off LIDAR updates
    }

    // print the output
    cout << "z_ = " << ekf_.z_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
*/
}



void EKF::Predict() {
    //Use the state using the state transition matrix
    z_ = F_ * z_;
    //Update the covariance matrix using the process noise and state transition matrix
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void EKF::Update(const VectorXd &z) {

    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;

    VectorXd y = z - H_ * z_;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd K = PHt * S.inverse();

    //Update State
    z_ = z_ + (K * y);
    //Update covariance matrix
    long x_size = z_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_) * P_;

}

void EKF::UpdateEKF(const VectorXd &z) {

    float px = z_(0);
    float py = z_(1);
    float vx = z_(2);
    float vy = z_(3);

    //Convert the predictions into polar coordinates
    float rho_p = sqrt(px*px + py*py);
    float theta_p = atan2(py,px);

    if (rho_p < 0.0001){
        std::cout << "Small prediction value - reassigning Rho_p to 0.0005 to avoid division by zero";
        rho_p = 0.0001;
    }

    float rho_dot_p = (px*vx + py*vy)/rho_p;

    VectorXd z_pred = VectorXd(3);
    z_pred << rho_p, theta_p, rho_dot_p;

    VectorXd y = z - z_pred;

    //Adjust the value of theta if it is outside of [-PI, PI]
    if (y(1) > PI){
        y(1) = y(1) - 2*PI;
    }

    else if (y(1) < -PI){
        y(1) = y(1) + 2*PI;
    }

    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;

    MatrixXd S = H_ * PHt + R_;
    MatrixXd K = PHt * S.inverse();

    //Update State
    z_ = z_ + (K * y);
    //Update covariance matrix
    long x_size = z_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_) * P_;
}

MatrixXd EKF::CalculateFpred(Camera left, Camera right, Camera leftNxt)
{
    Eigen::MatrixXd Fpred = MatrixXd::Identity(stateSize, stateSize);

    Fpred.topLeftCorner(6,6).setZero();

    return Fpred;
}

MatrixXd EKF::CalculateBpred(Camera left, Camera right, Camera leftNxt)
{
    Eigen::MatrixXd Bpred = MatrixXd::Zero(stateSize, stateSize);

    Bpred.topLeftCorner(6,6).setIdentity();

    return Bpred;
}

MatrixXd EKF::X_hat(Camera left, Camera right, Camera leftNxt) {

    int noOfMatches = left.currKpts.size();

    Eigen::VectorXd X_hat = Eigen::VectorXd(noOfMatches*6);

    for (int i = 0; i < noOfMatches; ++i) {
        X_hat[i] = left.currKpts[i].pt.x;
        X_hat[i+1] = left.currKpts[i].pt.y;
        X_hat[i+2] = right.currKpts[i].pt.x;
        X_hat[i+3] = right.currKpts[i].pt.y;
        X_hat[i+4] = leftNxt.currKpts[i].pt.x;
        X_hat[i+5] = leftNxt.currKpts[i].pt.y;
    }
}

MatrixXd EKF::Constraint( Camera left, Camera right, Camera leftNxt) {

    int noOfMatches = left.currKpts.size();
    int cPM = 3;  // constraints per match

    MatrixXd A = Eigen::MatrixXd(noOfMatches * cPM, stateSize);

    for (int j = 0; j < last_state ; j++) {
        // Loop through state

        for (int i = 0; i < noOfMatches; i ++) {
            // Loop through all keypoints

            Eigen::Vector3d leftKpt = Eigen::Vector3d({ left.currKpts[i].pt.x, left.currKpts[i].pt.y, 1});
            Eigen::Vector3d rightKpt = Eigen::Vector3d({ right.currKpts[i].pt.x, right.currKpts[i].pt.y, 1});
            Eigen::Vector3d nxtKpt = Eigen::Vector3d({ leftNxt.currKpts[i].pt.x, leftNxt.currKpts[i].pt.y, 1});


            A(i * cPM, j) = CalculateGqr(leftKpt, rightKpt, nxtKpt, 1, 1);
            A(i * cPM + 1, j) = CalculateGqr(leftKpt, rightKpt, nxtKpt, 1, 2);
            A(i * cPM +2, j) = leftKpt.transpose() * Fundamental * rightKpt;
        }
    }

    return A;
}

double EKF::CalculateGqr(Eigen::Vector3d leftKpt, Eigen::Vector3d rightKpt, Eigen::Vector3d nextKpt,
                        int q, int r) {

    double value = 0;

    for (int l = 0; l < 3; ++l) {
        value +=  leftKpt(l) * ( rightKpt(q) * nextKpt(r) * triTensor(Pleft, Pright, PleftNxt, l, 3, 3)
                - nextKpt(r) * triTensor(Pleft, Pright, PleftNxt, l, q, 3)
                - rightKpt(q) * triTensor(Pleft, Pright, PleftNxt, l, 3, r)
                + triTensor(Pleft, Pright, PleftNxt, l, q, r) );
    }

    return value;
}



MatrixXd EKF::CalculateA( Camera left, Camera right, Camera leftNxt) {

    MatrixXd dF = Eigen::Matrix<double, 3, 3>::Zero();

    int noOfMatches = left.currKpts.size();
    int cPM = 3;  // constraints per match

    MatrixXd A = Eigen::MatrixXd(noOfMatches * cPM, stateSize);

    for (int j = 0; j < last_state ; j++) {
        // Loop through state

        UpdateDerivProj(j);
        dF = derivateF(j);

        for (int i = 0; i < noOfMatches; i ++) {
            // Loop through all keypoints

            Eigen::Vector3d leftKpt = Eigen::Vector3d({ left.currKpts[i].pt.x, left.currKpts[i].pt.y, 1});
            Eigen::Vector3d rightKpt = Eigen::Vector3d({ right.currKpts[i].pt.x, right.currKpts[i].pt.y, 1});
            Eigen::Vector3d nxtKpt = Eigen::Vector3d({ leftNxt.currKpts[i].pt.x, leftNxt.currKpts[i].pt.y, 1});

            for (int l = 0; l < 3; ++l) {
                A(i* cPM, j) += leftKpt(l) * (rightKpt(1) * nxtKpt(1) * dT_lqr(left, right, leftNxt, l, 3, 3)
                                         - nxtKpt(1) * dT_lqr(left, right, leftNxt, l, 1, 3)
                                         - rightKpt(1) * dT_lqr(left, right, leftNxt, l, 3, 1)
                                         + dT_lqr(left, right, leftNxt, l, 1, 1));

                A(i* cPM + 1, j) += leftKpt(l) * (rightKpt(1) * nxtKpt(2) * dT_lqr(left, right, leftNxt, l, 3, 3)
                                             - nxtKpt(2) * dT_lqr(left, right, leftNxt, l, 1, 3)
                                             - rightKpt(1) * dT_lqr(left, right, leftNxt, l, 3, 2)
                                             + dT_lqr(left, right, leftNxt, l, 1, 2));
            }

            A(i* cPM +2, j) = leftKpt.transpose() * dF * rightKpt;
        }
    }

    return A;
}

MatrixXd EKF::CalculateB( Camera left, Camera right, Camera leftNxt) {

    int noOfMatches= left.currKpts.size();
    int cPM = 3; // Constraints per match
    int dPM = 6; // Derivatives per match

    MatrixXd B = Eigen::MatrixXd(noOfMatches * cPM, noOfMatches * dPM);

    UpdateProjAndFundamental();

    // Block diagonal

    for (int i = 0; i < noOfMatches; ++i) {
        // Loop through all keypoints

        Eigen::Vector3d leftKpt = Eigen::Vector3d({left.currKpts[i].pt.x, left.currKpts[i].pt.y, 1});
        Eigen::Vector3d rightKpt = Eigen::Vector3d({right.currKpts[i].pt.x, right.currKpts[i].pt.y, 1});
        Eigen::Vector3d nxtKpt = Eigen::Vector3d({leftNxt.currKpts[i].pt.x, leftNxt.currKpts[i].pt.y, 1});

        for (int j = 0; j < last_xd; j++) {
            // Loop through kpts

            B(i * cPM, i * dPM + j) = derivateGqr(leftKpt, rightKpt, nxtKpt, 1, 1, j);
            B(i * cPM + 1, i * dPM + j) = derivateGqr(leftKpt, rightKpt, nxtKpt, 1, 1, j);
            B(i * cPM + 2, i * dPM + j) = derivateHe(leftKpt, rightKpt, j);
        }
    }
    return B;
}

double EKF::derivateHe(Eigen::Vector3d leftKpt, Eigen::Vector3d rightKpt, int param) {

    Eigen::Vector3d xl, xr;

    switch (param) {
        case Px_l:
            xl = {1, 0, 0};
            xr = rightKpt;

            break;

        case Py_l:
            xl = {0, 1, 0};
            xr = rightKpt;

            break;

        case Px_r:
            xl = rightKpt;
            xr = {1, 0, 0};
            break;

        case Py_r:
            xl = leftKpt;
            xr = {0, 1, 0};

        default:
            break;

    }

    return xl.transpose() * Fundamental * xr;

}

double EKF::derivateGqr(Eigen::Vector3d leftKpt, Eigen::Vector3d rightKpt, Eigen::Vector3d nextKpt,
                        int q, int r, int param) {

    double value = 0;

    switch (param) {
        case Px_l:
            value = rightKpt(q) * nextKpt(r) * triTensor(Pleft, Pright, PleftNxt, 1, 3, 3)
                    - nextKpt(r) * triTensor(Pleft, Pright, PleftNxt, 1, q, 3)
                    - rightKpt(q) * triTensor(Pleft, Pright, PleftNxt, 1, 3, r)
                    + triTensor(Pleft, Pright, PleftNxt, 1, q, r);
            break;

        case Py_l:
            value = rightKpt(q) * nextKpt(r) * triTensor(Pleft, Pright, PleftNxt, 2, 3, 3)
                    - nextKpt(r) * triTensor(Pleft, Pright, PleftNxt, 2, q, 3)
                    - rightKpt(q) * triTensor(Pleft, Pright, PleftNxt, 2, 3, r)
                    + triTensor(Pleft, Pright, PleftNxt, 2, q, r);

            break;

        case Px_r:
            for (int l = 0; l < 3; ++l) {
                value +=  leftKpt(l) * ( nextKpt(r) * triTensor(Pleft, Pright, PleftNxt, l, 3, 3)
                        - triTensor(Pleft, Pright, PleftNxt, l, 3, r));
            }
            break;

        case Px_nxt:
            if (r == 0) {
                for (int l = 0; l < 3; ++l) {
                    value +=  leftKpt(l) * ( rightKpt(q) * triTensor(Pleft, Pright, PleftNxt, l, 3, 3)
                            - triTensor(Pleft, Pright, PleftNxt, l, q, 3));
                }
            }
            break;

        case Py_nxt:
            if (r == 1) {
                for (int l = 0; l < 3; ++l) {
                    value +=  leftKpt(l) * ( rightKpt(q) * triTensor(Pleft, Pright, PleftNxt, l, 3, 3)
                                             - triTensor(Pleft, Pright, PleftNxt, l, q, 3));
                }
            }
            break;

        default:
            break;

    }

    return value;
}

void EKF::UpdateProjAndFundamental() {

    Pleft <<    1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0;

    Pright <<   1, 0, 0, - baseline[0],
                0, 1, 0, - baseline[1],
                0, 0, 1, - baseline[2];


    PleftNxt << 1, 0, 0, - z_[epochTx],
                0, 1, 0, - z_[epochTy],
                0, 0, 1, - z_[epochTz];

    MatrixXd Kl = Eigen::Matrix<double, 3, 3>::Zero();
    MatrixXd Kr = Eigen::Matrix<double, 3, 3>::Zero();
    MatrixXd Rright;;
    MatrixXd Rnxt;

    Kl <<   z_[fxL], 0, z_[cxL],
            0, z_[fyL], z_[cyL],
            0, 0, 1;

    Kr <<   z_[fxR], 0, z_[cxR],
            0, z_[fyR], z_[cyR],
            0, 0, 1;

    Rright = GetRotMatrix(z_[extR], z_[extP], z_[extY]);

    Pleft = Kl * Pleft;
    Pright = Kr * Rright * Pright;
    PleftNxt = Kr * Rnxt * PleftNxt;

    Fundamental = Kl.transpose().inverse() * Skew({z_[extR], z_[extP], z_[extY]})
            * Rright.transpose() * Kr.inverse();

}


void EKF::UpdateDerivProj(int param) {
    dPleft = derivatePa(param);
    dPright = derivatePb(param);
    dPleftNxt = derivatePc(param);
}


double EKF::dT_lqr( Camera left, Camera right, Camera leftNxt, int l, int q, int r)
{
    Eigen::Matrix4d Tmat = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d dTmat = Eigen::Matrix4d::Zero();

    int k = 0;
    for (int i = 0; i < 3; i++) {
        if ( i != l-1) {
            Tmat.row(i) = left.model.P.row(k);
            dTmat.row(i) = dPleft.row(k);
            k++;
        }
    }
    Tmat.row(2) = right.model.P.row(q);
    Tmat.row(3) = leftNxt.model.P.row(r);

    dTmat.row(2) = dPright.row(q);
    dTmat.row(3) = dPleftNxt.row(r);

    double dt = pow(-1, l+1) * Tmat.determinant();

    dTmat = Tmat.inverse() * dTmat;

    return dt * (Tmat.inverse() * dTmat).trace();



}

double EKF::triTensor(Matrixf34 Pa, Matrixf34 Pb, Matrixf34 Pc, int l, int q, int r)
{
    Eigen::Matrix4d Tmat = Eigen::Matrix4d::Zero();


    int k = 0;
    for (int i = 0; i < 3; i++) {
        if ( i != l-1) {
            Tmat.row(i) = Pa.row(k);
            k++;
        }
    }
    Tmat.row(2) = Pb.row(q);
    Tmat.row(3) = Pc.row(r);

    double T = Tmat.determinant();

    T = pow(-1, l+1) * T;


    std::cout << T << std::endl;

    return T;
}


MatrixXd EKF::CalculateJacobian(const VectorXd& x_state) {

    //Initalize the Jacobian
    MatrixXd Hj(3,4);
    //Retrive the state values from the vector
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //Set the Jacobian to zero
    Hj << 0,0,0,0,
            0,0,0,0,
            0,0,0,0;

    //Check for small values of position magnitude to avoid division by zero
    float rho = pow((pow(px,2) + pow(py,2)), 0.5);
    if( rho < 0.0001){
        cout << "Value of rho too small - possible div by 0. Reassigning rho = 0.0005";
        rho = 0.0001;
    }

    float inv_rho = pow(rho, -1);
    Hj(0,0) = px * inv_rho;
    Hj(1,0) = -py * pow(inv_rho,2);
    Hj(2,0) = py * (vx*py - vy*px) * pow(inv_rho, 3);
    Hj(0,1) = py * inv_rho;
    Hj(1,1) = px * pow(inv_rho, 2);
    Hj(2,1) = px * (vy*px - vx*py) * pow(inv_rho,3);
    Hj(2,2) = Hj(0,0);
    Hj(2,3) = Hj(0,1);

    return Hj;

}

MatrixXd EKF::derivateF(int param)
{
    MatrixXd dF = Eigen::Matrix<double, 3, 3>::Zero();

    MatrixXd Cskew = Skew(baseline);

    MatrixXd Kl = Eigen::Matrix<double, 3, 3>::Zero();
    MatrixXd Kr = Eigen::Matrix<double, 3, 3>::Zero();
    MatrixXd Rot = Eigen::Matrix<double, 3, 3>::Zero();

    Kl <<   z_[fxL], 0, z_[cxL],
            0, z_[fyL], z_[cyL],
            0, 0, 1;

    Kr <<   z_[fxR], 0, z_[cxR],
            0, z_[fyR], z_[cyR],
            0, 0, 1;


    switch (param) {

        case fxL:
            Kl <<   1, 0, 0,
                    0, 0, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);
            break;

        case fyL:
            Kl <<   0, 0, 0,
                    0, 1, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);
            break;

        case cxL:
            Kl <<   0, 0, 1,
                    0, 0, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);
            break;

        case cyL:
            Kl <<   0, 0, 0,
                    0, 0, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);
            break;

        case fxR:
            Kr <<   1, 0, 0,
                    0, 0, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);
            break;

        case fyR:
            Kr <<   0, 0, 0,
                    0, 1, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);
            break;

        case cxR:
            Kr <<   0, 0, 1,
                    0, 0, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);
            break;

        case cyR:
            Kr <<   0, 0, 0,
                    0, 0, 1,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);
            break;

        case extR:

            Rot(0,0) = - sin(z_[extR]) * cos(z_[extY])
                       - cos(z_[extR]) * sin(z_[extP]) * sin(z_[extY]);
            Rot(0,1) = - cos(z_[extR]) * cos(z_[extP]);
            Rot(0,2) = - sin(z_[extR]) * sin(z_[extY])
                       + cos(z_[extR]) * sin(z_[extP]) * cos(z_[extY]);

            Rot(1,0) = cos(z_[extR]) * cos(z_[extY])
                       - sin(z_[extR]) * sin(z_[extP]) * sin(z_[extY]);
            Rot(1,1) = - sin(z_[extR]) * cos(z_[extP]);
            Rot(1,2) = cos(z_[extR]) * sin(z_[extY])
                       + sin(z_[extR]) * sin(z_[extP]) * cos(z_[extY]);

            Rot(2,0) = 0;
            Rot(2, 1) = 0;
            Rot(2,2) = 0;

            break;

        case extP:

            Rot(0,0) = sin(z_[extR]) * cos(z_[extP]) * sin(z_[extY]);
            Rot(0,1) = sin(z_[extR]) * sin(z_[extP]);
            Rot(0,2) = sin(z_[extR]) * cos(z_[extP]) * cos(z_[extY]);

            Rot(1,0) = cos(z_[extR]) * cos(z_[extP]) * sin(z_[extY]);
            Rot(1,1) = - cos(z_[extR]) * sin(z_[extP]);
            Rot(1,2) = cos(z_[extR]) * cos(z_[extP]) * cos(z_[extY]);

            Rot(2,0) = sin(z_[extP]) * sin(z_[extY]);
            Rot(2,1) = cos(z_[extP]);
            Rot(2,2) = - sin(z_[extP]) * cos(z_[extY]);
            break;

        case extY:

            Rot(0,0) = - cos(z_[extR]) * sin(z_[extY])
                       - sin(z_[extR]) * sin(z_[extP]) * cos(z_[extY]);
            Rot(0,1) = 0;
            Rot(0,2) = cos(z_[extR]) * cos(z_[extY])
                       - sin(z_[extR]) * sin(z_[extP]) * sin(z_[extY]);

            Rot(1,0) = - sin(z_[extR]) * sin(z_[extY])
                       + cos(z_[extR]) * sin(z_[extP]) * cos(z_[extY]);
            Rot(1,1) = 0;
            Rot(1,2) = sin(z_[extR]) * cos(z_[extY])
                       + cos(z_[extR]) * sin(z_[extP]) * sin(z_[extY]);

            Rot(2,0) = - cos(z_[extP]) * cos(z_[extY]);
            Rot(2,1) = 0;
            Rot(2,2) = - cos(z_[extP]) * sin(z_[extY]);

            break;

        default:

            break;

    }


    dF = Kl.transpose().inverse() * Cskew * Rot.transpose() * Kr.inverse();
}

Eigen::MatrixXd EKF::GetRotMatrix(double roll, double pitch, double yaw) {

    MatrixXd Rot = Eigen::Matrix<double, 3, 3>::Zero();


    Rot(0,0) = cos(roll) * cos(yaw) - sin(roll) * sin(pitch) * sin(yaw);
    Rot(0,1) = - sin(roll) * cos(pitch);
    Rot(0,2) = cos(roll) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw);

    Rot(1,0) = sin(roll) * cos(yaw) + cos(roll) * sin(pitch) * sin(yaw);
    Rot(1,1) = cos(roll) * cos(pitch);
    Rot(1,2) = sin(roll) * sin(yaw) - cos(roll) * sin(pitch) * cos(yaw);

    Rot(2,0) = - cos(pitch) * sin(yaw);
    Rot(2,1) = sin(pitch);
    Rot(2,2) = cos(pitch) * cos(yaw);

    return Rot;
}


Eigen::Matrix3d EKF::Skew(Eigen::Vector3f v)
{
    Eigen::Matrix3d skew;
    skew << 0,      -v[2],  v[1],
            v[2],   0,      -v[0],
            -v[1],  v[0],   0;
    return skew;
}


MatrixXd EKF::derivatePa(int param)
{
    MatrixXd dPa = Eigen::Matrix<double, 4, 4>::Zero();

    switch (param) {
        case fxL:
            dPa(0, 0) = 1;
            break;

        case fyL:
            dPa(1, 1) = 1;
            break;

        case cxL:
            dPa(0, 2) = 1;
            break;

        case cyL:
            dPa(1, 2) = 1;
            break;

        default:
            break;
    }

    return dPa;
}

MatrixXd EKF::derivatePb(int param)
{
    MatrixXd dPb = Eigen::Matrix<double, 4, 4>::Zero();
    MatrixXd Rot = Eigen::Matrix<double, 3,3>::Zero();
    MatrixXd Kr = Eigen::Matrix<double, 3, 3>::Zero();
    MatrixXd P = Eigen::Matrix<double, 3, 4>::Zero();

    Kr <<   z_[fxR], 0, z_[cxR],
            0, z_[fyR], z_[cyR],
            0, 0, 1;

    P <<   1, 0, 0, -baseline(0),
            0, 1, 0, -baseline(1),
            0, 0, 1, -baseline(2);

    switch (param) {

        case fxR:
            Kr <<   1, 0, 0,
                    0, 0, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);

            break;

        case fyR:
            Kr <<   0, 0, 0,
                    0, 1, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);
            break;

        case cxR:
            Kr <<   0, 0, 1,
                    0, 0, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);

            break;

        case cyR:
            Kr <<   0, 0, 0,
                    0, 0, 1,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[extR], z_[extP], z_[extY]);

            break;

        case extR:

            Rot(0,0) = - sin(z_[extR]) * cos(z_[extY])
                       - cos(z_[extR]) * sin(z_[extP]) * sin(z_[extY]);
            Rot(0,1) = - cos(z_[extR]) * cos(z_[extP]);
            Rot(0,2) = - sin(z_[extR]) * sin(z_[extY])
                       + cos(z_[extR]) * sin(z_[extP]) * cos(z_[extY]);

            Rot(1,0) = cos(z_[extR]) * cos(z_[extY])
                       - sin(z_[extR]) * sin(z_[extP]) * sin(z_[extY]);
            Rot(1,1) = - sin(z_[extR]) * cos(z_[extP]);
            Rot(1,2) = cos(z_[extR]) * sin(z_[extY])
                       + sin(z_[extR]) * sin(z_[extP]) * cos(z_[extY]);

            Rot(2,0) = 0;
            Rot(2, 1) = 0;
            Rot(2,2) = 0;

            break;

        case extP:

            Rot(0,0) = sin(z_[extR]) * cos(z_[extP]) * sin(z_[extY]);
            Rot(0,1) = sin(z_[extR]) * sin(z_[extP]);
            Rot(0,2) = sin(z_[extR]) * cos(z_[extP]) * cos(z_[extY]);

            Rot(1,0) = cos(z_[extR]) * cos(z_[extP]) * sin(z_[extY]);
            Rot(1,1) = - cos(z_[extR]) * sin(z_[extP]);
            Rot(1,2) = cos(z_[extR]) * cos(z_[extP]) * cos(z_[extY]);

            Rot(2,0) = sin(z_[extP]) * sin(z_[extY]);
            Rot(2,1) = cos(z_[extP]);
            Rot(2,2) = - sin(z_[extP]) * cos(z_[extY]);

            break;

        case extY:

            Rot(0,0) = - cos(z_[extR]) * sin(z_[extY])
                       - sin(z_[extR]) * sin(z_[extP]) * cos(z_[extY]);
            Rot(0,1) = 0;
            Rot(0,2) = cos(z_[extR]) * cos(z_[extY])
                       - sin(z_[extR]) * sin(z_[extP]) * sin(z_[extY]);

            Rot(1,0) = - sin(z_[extR]) * sin(z_[extY])
                       + cos(z_[extR]) * sin(z_[extP]) * cos(z_[extY]);
            Rot(1,1) = 0;
            Rot(1,2) = sin(z_[extR]) * cos(z_[extY])
                       + cos(z_[extR]) * sin(z_[extP]) * sin(z_[extY]);

            Rot(2,0) = - cos(z_[extP]) * cos(z_[extY]);
            Rot(2,1) = 0;
            Rot(2,2) = - cos(z_[extP]) * sin(z_[extY]);

            break;

        default:
            break;

    }

    dPb = Kr * Rot * P;

    return dPb;
}


MatrixXd EKF::derivatePc(int param)
{

    MatrixXd dPc = Eigen::Matrix<double, 4, 4>::Zero();
    MatrixXd Rot = Eigen::Matrix<double, 3,3>::Zero();
    MatrixXd Kl = Eigen::Matrix<double, 3, 3>::Zero();
    MatrixXd P = Eigen::Matrix<double, 3, 4>::Zero();


    Kl <<   z_[fxL], 0, z_[cxL],
            0, z_[fyL], z_[cyL],
            0, 0, 1;

    P <<    1, 0, 0, -z_[epochTx],
            0, 1, 0, -z_[epochTy],
            0, 0, 1, -z_[epochTz];


    switch (param) {
        case fxL:
            Kl <<   1, 0, 0,
                    0, 0, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[epochR], z_[epochP], z_[epochY]);

            break;

        case fyL:
            Kl <<   0, 0, 0,
                    0, 1, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[epochR], z_[epochP], z_[epochY]);

            break;


        case cxL:
            Kl <<   0, 0, 1,
                    0, 0, 0,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[epochR], z_[epochP], z_[epochY]);

            break;

        case cyL:
            Kl <<   0, 0, 0,
                    0, 0, 1,
                    0, 0, 0;

            Rot = GetRotMatrix(z_[epochR], z_[epochP], z_[epochY]);

            break;

        case epochR:

            Rot(0,0) = - sin(z_[epochR]) * cos(z_[epochY])
                       - cos(z_[epochR]) * sin(z_[epochP]) * sin(z_[epochY]);
            Rot(0,1) = - cos(z_[epochR]) * cos(z_[epochP]);
            Rot(0,2) = - sin(z_[epochR]) * sin(z_[epochY])
                       + cos(z_[epochR]) * sin(z_[epochP]) * cos(z_[epochY]);

            Rot(1,0) = cos(z_[epochR]) * cos(z_[epochY])
                       - sin(z_[epochR]) * sin(z_[epochP]) * sin(z_[epochY]);
            Rot(1,1) = - sin(z_[epochR]) * cos(z_[epochP]);
            Rot(1,2) = cos(z_[epochR]) * sin(z_[epochY])
                       + sin(z_[epochR]) * sin(z_[epochP]) * cos(z_[epochY]);

            Rot(2,0) = 0;
            Rot(2, 1) = 0;
            Rot(2,2) = 0;


            break;

        case epochP:

            Rot(0,0) = sin(z_[epochR]) * cos(z_[epochP]) * sin(z_[epochY]);
            Rot(0,1) = sin(z_[epochR]) * sin(z_[epochP]);
            Rot(0,2) = sin(z_[epochR]) * cos(z_[epochP]) * cos(z_[epochY]);

            Rot(1,0) = cos(z_[epochR]) * cos(z_[epochP]) * sin(z_[epochY]);
            Rot(1,1) = - cos(z_[epochR]) * sin(z_[epochP]);
            Rot(1,2) = cos(z_[epochR]) * cos(z_[epochP]) * cos(z_[epochY]);

            Rot(2,0) = sin(z_[epochP]) * sin(z_[epochY]);
            Rot(2,1) = cos(z_[epochP]);
            Rot(2,2) = - sin(z_[epochP]) * cos(z_[epochY]);

            break;

        case epochY:

            Rot(0,0) = - cos(z_[epochR]) * sin(z_[epochY])
                       - sin(z_[epochR]) * sin(z_[epochP]) * cos(z_[epochY]);
            Rot(0,1) = 0;
            Rot(0,2) = cos(z_[epochR]) * cos(z_[epochY])
                       - sin(z_[epochR]) * sin(z_[epochP]) * sin(z_[epochY]);

            Rot(1,0) = - sin(z_[epochR]) * sin(z_[epochY])
                       + cos(z_[epochR]) * sin(z_[epochP]) * cos(z_[epochY]);
            Rot(1,1) = 0;
            Rot(1,2) = sin(z_[epochR]) * cos(z_[epochY])
                       + cos(z_[epochR]) * sin(z_[epochP]) * sin(z_[epochY]);

            Rot(2,0) = - cos(z_[epochP]) * cos(z_[epochY]);
            Rot(2,1) = 0;
            Rot(2,2) = - cos(z_[epochP]) * sin(z_[epochY]);


            break;

        case epochTx:
            P <<    0, 0, 0, -1,
                    0, 0, 0, 0,
                    0, 0, 0, 0;

            Rot = GetRotMatrix(z_[epochR], z_[epochP], z_[epochY]);

            break;

        case epochTy:
            P <<    0, 0, 0, 0,
                    0, 0, 0, -1,
                    0, 0, 0, 0;

            Rot = GetRotMatrix(z_[epochR], z_[epochP], z_[epochY]);

            break;

        case epochTz:
            P <<    0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, -1;

            Rot = GetRotMatrix(z_[epochR], z_[epochP], z_[epochY]);

            break;

        default:
            break;
    }

    dPc = Kl * Rot * P;

    return dPc;
}




/*
VectorXd EKF::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    //Validate the estimations vector
    if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
        cout<<"Error in size of Estimations vector or size mismatch with Ground Truth vector";
        return rmse;
    }

    //Accumulate the residual
    for(int i = 0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        rmse = rmse + (residual.array() * residual.array()).matrix();
    }

    //Mean and Sqrt the error
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
}
*/
