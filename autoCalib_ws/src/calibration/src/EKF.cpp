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
    z_[0] = 0; //x
    z_[1] = 0; //y
    z_[2] = 0; //z
    z_[3] = 0; //roll
    z_[4] = 0; //pitch
    z_[5] = 0; //yaw
    //Extrinsics
    z_[6] = camRight.model.rEuler[0]; //roll
    z_[7] = camRight.model.rEuler[1]; //pich
    z_[8] = camRight.model.rEuler[2]; //yaw
    //Left Camera
    z_[9] = camRight.model.fx;
    z_[10] = camRight.model.fy;
    z_[11] = camRight.model.cx;
    z_[12] = camRight.model.cy;
    z_[13] = 0;    //Distortion
    z_[14] = 0;    //Distortion
    //Right Camera
    z_[15] = camRight.model.fx;
    z_[16] = camRight.model.fy;
    z_[17] = camRight.model.cx;
    z_[18] = camRight.model.cy;
    z_[19] = 0;    //Distortion
    z_[20] = 0;    //Distortion

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

MatrixXd EKF::CalculateA(Camera leftNxt, Camera left, Camera right)
{
    for (int i = 0; i < left.currKpts.size(); ++i) {


    }

}

MatrixXd EKF::derivateA1(String param)
{
    MatrixXd A1 = Eigen::Matrix<double, 1, 4>::Zero();

    if (param == "fx") {
        A1(0) = 1;
    }
    else if (param == "cy") {
        A1(2) = 1;
    }

    return A1;
}

MatrixXd EKF::derivateA2(String param)
{
    MatrixXd A2 = Eigen::Matrix<double, 1, 4>::Zero();

    if (param == "fy") {
        A2(1) = 1;
    }
    else if (param == "cy") {
        A2(2) = 1;
    }

    return A2;
}

MatrixXd EKF::derivateA3(String param)
{
    MatrixXd A3 = Eigen::Matrix<double, 1, 4>::Zero();

    return A3;
}

MatrixXd EKF::derivateB1(String param)
{

    MatrixXd B1 = Eigen::Matrix<double, 1, 4>::Zero();

    if( param == "fx"){
        B1(0) = cos(z_[8])* cos(z_[7]);
        B1(1) = - sin(z_[8]) * cos(z_[6]) + cos(z_[8]) * sin(z_[7]) * sin(z_[6]);
        B1(2) = sin (z_[8]) * sin(z_[6]) + cos(z_[8]) * sin(z_[7]) * cos(z_[6]);
        B1(3) = 0; //baselineX
    }

    if (param == "cx")
    {
        B1(0) = - sin(z_[7]);
        B1(1) = cos(z_[8]) * sin(z_[6]);
        B1(2) = cos(z_[7]) * cos(z_[6]);
        B1(3) = 0; //baselineZ
    }

    //Vinkler
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
