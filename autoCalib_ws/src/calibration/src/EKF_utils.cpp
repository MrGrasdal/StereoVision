//
// Created by martin on 07.05.2021.
//

#include "EKF_utils.h"



MatrixXd EKF_utils::CalculateFpred(Camera left, Camera right, Camera leftNxt)
{
    Eigen::MatrixXd Fpred = MatrixXd::Identity(stateSize, stateSize);

    Fpred.topLeftCorner(6,6).setZero();

    return Fpred;
}

MatrixXd EKF_utils::CalculateBpred(Camera left, Camera right, Camera leftNxt)
{
    Eigen::MatrixXd Bpred = MatrixXd::Zero(stateSize, stateSize);

    Bpred.topLeftCorner(6,6).setIdentity();

    return Bpred;
}

MatrixXd EKF_utils::X_hat(Camera left, Camera right, Camera leftNxt) {

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

MatrixXd EKF_utils::Constraint( Camera left, Camera right, Camera leftNxt) {

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

double EKF_utils::CalculateGqr(Eigen::Vector3d leftKpt, Eigen::Vector3d rightKpt, Eigen::Vector3d nextKpt,
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



MatrixXd EKF_utils::CalculateA( Camera left, Camera right, Camera leftNxt) {

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

MatrixXd EKF_utils::CalculateB( Camera left, Camera right, Camera leftNxt) {

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

double EKF_utils::derivateHe(Eigen::Vector3d leftKpt, Eigen::Vector3d rightKpt, int param) {

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

double EKF_utils::derivateGqr(Eigen::Vector3d leftKpt, Eigen::Vector3d rightKpt, Eigen::Vector3d nextKpt,
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

void EKF_utils::UpdateProjAndFundamental() {

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


void EKF_utils::UpdateDerivProj(int param) {
    dPleft = derivatePa(param);
    dPright = derivatePb(param);
    dPleftNxt = derivatePc(param);
}


double EKF_utils::dT_lqr( Camera left, Camera right, Camera leftNxt, int l, int q, int r)
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

double EKF_utils::triTensor(Matrixf34 Pa, Matrixf34 Pb, Matrixf34 Pc, int l, int q, int r)
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


MatrixXd EKF_utils::CalculateJacobian(const VectorXd& x_state) {

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

MatrixXd EKF_utils::derivateF(int param)
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

Eigen::MatrixXd EKF_utils::GetRotMatrix(double roll, double pitch, double yaw) {

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


Eigen::Matrix3d EKF_utils::Skew(Eigen::Vector3f v)
{
    Eigen::Matrix3d skew;
    skew << 0,      -v[2],  v[1],
            v[2],   0,      -v[0],
            -v[1],  v[0],   0;
    return skew;
}


MatrixXd EKF_utils::derivatePa(int param)
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

MatrixXd EKF_utils::derivatePb(int param)
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


MatrixXd EKF_utils::derivatePc(int param, Eigen::VectorXd *state)
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