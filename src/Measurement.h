//
// Created by nikita on 08.05.19.
//

#ifndef EXTENDED_KALMAN_FILTER_MEASUREMENT_H
#define EXTENDED_KALMAN_FILTER_MEASUREMENT_H

#include "Eigen/Dense"

class Measurement {
public:
    enum SensorType{
        LASER,
        RADAR
    } sensor_type;

    long long timestamp;

    Eigen::VectorXd raw_measurements;
};


#endif //EXTENDED_KALMAN_FILTER_MEASUREMENT_H
