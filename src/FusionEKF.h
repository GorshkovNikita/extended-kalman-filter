//
// Created by nikita on 09.05.19.
//

#ifndef EXTENDED_KALMAN_FILTER_FUSIONEKF_H
#define EXTENDED_KALMAN_FILTER_FUSIONEKF_H

#include "KalmanFilter.h"
#include "Measurement.h"

class FusionEKF {
public:
    FusionEKF();
    virtual ~FusionEKF();

    void processMeasurement(Measurement& measurement);

    KalmanFilter kalmanFilter;

private:
    bool is_initialized;

    long long previous_timestamp;

    Eigen::MatrixXd R_laser;
    Eigen::MatrixXd R_radar;
    Eigen::MatrixXd H_laser;
    Eigen::MatrixXd Hj;
    float noise_ax;
    float noise_ay;
};


#endif //EXTENDED_KALMAN_FILTER_FUSIONEKF_H
