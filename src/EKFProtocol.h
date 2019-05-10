//
// Created by nikita on 08.05.19.
//

#ifndef EXTENDED_KALMAN_FILTER_EKFPROTOCOL_H
#define EXTENDED_KALMAN_FILTER_EKFPROTOCOL_H

#include <string_view>
#include <sstream>
#include "Eigen/Dense"
#include "Measurement.h"
#include "FusionEKF.h"

class EKFProtocol {
public:
    EKFProtocol();

    std::string processMessage(std::string_view message);

private:
    std::string extractPayload(const std::string& message);
    Measurement parseMeasurement(std::istringstream& measurementStream);
    void parseGroundTruth(std::istringstream& measurementStream);
    std::string estimate(Measurement& measurement);

    std::vector<Eigen::VectorXd> estimations;
    std::vector<Eigen::VectorXd> ground_truth;
    FusionEKF fusionEkf;
};


#endif //EXTENDED_KALMAN_FILTER_EKFPROTOCOL_H
