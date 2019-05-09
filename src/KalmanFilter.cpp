//
// Created by nikita on 09.05.19.
//

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in, Eigen::MatrixXd &H_in,
                        Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in) {
    x = x_in;
    P = P_in;
    F = F_in;
    H = H_in;
    R = R_in;
    Q = Q_in;
}

void KalmanFilter::predict() {

}

void KalmanFilter::update(const Eigen::VectorXd &z) {

}

void KalmanFilter::updateEKF(const Eigen::VectorXd &z) {

}
