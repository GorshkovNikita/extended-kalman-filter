//
// Created by nikita on 09.05.19.
//

#include "FusionEKF.h"

FusionEKF::FusionEKF() {
    is_initialized = false;
    previous_timestamp = 0;

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

//    ???
//    kalmanFilter.init()

}

FusionEKF::~FusionEKF() = default;

void FusionEKF::processMeasurement(Measurement& measurement) {
    if (!is_initialized) {
        /**
         * TODO: Initialize the state ekf_.x_ with the first measurement.
         * TODO: Create the covariance matrix.
         * You'll need to convert radar from polar to cartesian coordinates.
         */

//        ???
//        kalmanFilter.init()

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
//        kalmanFilter.updateEKF();
    } else {
//        kalmanFilter.update();
    }
}
