//
// Created by nikita on 09.05.19.
//

#include "KalmanFilter.h"
#include "Eigen/Dense"
#include "Tools.h"
#include <iostream>

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::init() {
    x = Eigen::VectorXd(4);

    // state covariance matrix P
    P = Eigen::MatrixXd(4, 4);
    P <<    1,  0,  0,      0,
            0,  1,  0,      0,
            0,  0,  1000,   0,
            0,  0,  0,      1000;

    R = Eigen::MatrixXd(2, 2);
    H = Eigen::MatrixXd(2, 4);
    F = Eigen::MatrixXd(4, 4);
    Q = Eigen::MatrixXd(4, 4);
    I = Eigen::MatrixXd::Identity(x.size(), x.size());
}

void KalmanFilter::predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd &z) {
    Eigen::VectorXd y = z - H * x;
    Eigen::MatrixXd Ht = H.transpose();
    Eigen::MatrixXd S = H * P * Ht + R;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd K = P * Ht * Si;

    x = x + (K * y);
    P = (I - K * H) * P;
}

void KalmanFilter::updateEKF(const Eigen::VectorXd &z) {
    Eigen::VectorXd y = z - Tools::convertToPolar(x);
    Eigen::MatrixXd Ht = H.transpose();
    Eigen::MatrixXd S = H * P * Ht + R;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd K = P * Ht * Si;

    x = x + (K * y);
    P = (I - K * H) * P;
}
