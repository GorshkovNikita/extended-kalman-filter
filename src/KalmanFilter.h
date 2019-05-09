//
// Created by nikita on 09.05.19.
//

#ifndef EXTENDED_KALMAN_FILTER_KALMANFILTER_H
#define EXTENDED_KALMAN_FILTER_KALMANFILTER_H

#include "Eigen/Dense"

class KalmanFilter {
public:
    KalmanFilter();
    virtual ~KalmanFilter();

    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
              Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k and k+1 in s
     */
    void predict();

    /**
     * Updates the   state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void update(const Eigen::VectorXd &z);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void updateEKF(const Eigen::VectorXd &z);

    // state vector (px, py, vx, vy)
    Eigen::VectorXd x;

    // state covariance matrix
    Eigen::MatrixXd P;

    // state transition matrix
    Eigen::MatrixXd F;

    // process covariance matrix
    Eigen::MatrixXd Q;

    // measurement matrix
    Eigen::MatrixXd H;

    // measurement covariance matrix
    Eigen::MatrixXd R;
};


#endif //EXTENDED_KALMAN_FILTER_KALMANFILTER_H
