//
// Created by nikita on 09.05.19.
//

#include "FusionEKF.h"
#include "Tools.h"
#include <math.h>

FusionEKF::FusionEKF() {
    is_initialized = false;
    previous_timestamp = 0;
    noise_ax = 9;
    noise_ay = 9;

    R_laser = Eigen::MatrixXd(2, 2);
    R_radar = Eigen::MatrixXd(3, 3);
    H_laser = Eigen::MatrixXd(2, 4);
    Hj = Eigen::MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser <<  0.0225, 0,
                0,      0.0225;

    //measurement covariance matrix - radar
    R_radar <<  0.09,   0,      0,
                0,      0.0009, 0,
                0,      0,      0.09;

    /**
     * TODO: Finish initializing the FusionEKF.
     * TODO: Set the process and measurement noises
     */


    H_laser <<  1, 0, 0, 0,
                0, 1, 0, 0;

    kalmanFilter.init();

}

FusionEKF::~FusionEKF() = default;

void FusionEKF::processMeasurement(Measurement& measurement) {
    if (!is_initialized) {
        /**
         * TODO: Initialize the state ekf_.x_ with the first measurement.
         * TODO: Create the covariance matrix.
         * You'll need to convert radar from polar to cartesian coordinates.
         */

        if (measurement.sensor_type == Measurement::RADAR) {
            kalmanFilter.x << Tools::convertToCartesian(measurement.raw_measurements);
        } else {
            kalmanFilter.x <<   measurement.raw_measurements[0],
                                measurement.raw_measurements[1],
                                0,
                                0;
        }

        previous_timestamp = measurement.timestamp;
        is_initialized = true;
        return;
    }

    /**
     * Prediction
     */

    /**
     * TODO: Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * TODO: Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    float dt = (measurement.timestamp - previous_timestamp) / 1000000.0;
    previous_timestamp = measurement.timestamp;

    kalmanFilter.F <<   1,  0,  dt, 0,
                        0,  1,  0,  dt,
                        0,  0,  1,  0,
                        0,  0,  0,  1;

    kalmanFilter.Q <<   std::pow(dt, 4) * noise_ax / 4.0,  0,  std::pow(dt, 3) * noise_ax / 2.0,  0,
                        0,  std::pow(dt, 4) * noise_ay / 4.0,  0,  std::pow(dt, 3) * noise_ay / 2.0,
                        std::pow(dt, 3) * noise_ax / 2.0,  0,  std::pow(dt, 2) * noise_ax,  0,
                        0,  std::pow(dt, 3) * noise_ay / 2.0,  0,  std::pow(dt, 2) * noise_ay;

    kalmanFilter.predict();

    /**
     * Update
     */

    /**
     * TODO:
     * - Use the sensor type to perform the update step.
     * - Update the state and covariance matrices.
     */

    if (measurement.sensor_type == Measurement::RADAR) {
        kalmanFilter.R << R_radar;
        kalmanFilter.H << H_laser;
        kalmanFilter.updateEKF(measurement.raw_measurements);
    } else {
        kalmanFilter.R << R_laser;
        Hj << Tools::calculateJacobian(kalmanFilter.x);
        kalmanFilter.H << Hj;
        kalmanFilter.update(measurement.raw_measurements);
    }
}
