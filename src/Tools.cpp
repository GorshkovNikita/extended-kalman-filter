//
// Created by nikita on 09.05.19.
//

#include "Tools.h"

Eigen::VectorXd Tools::calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                     const std::vector<Eigen::VectorXd> &ground_truth) {
    if (estimations.empty() || estimations.size() != ground_truth.size()) {
        return Eigen::VectorXd();
    }
    int vectorSize = estimations[0].size();

    std::vector<Eigen::VectorXd> squaredResiduals(estimations.size());
    for (int i = 0; i < estimations.size(); i++) {
        Eigen::VectorXd residual = estimations[i] - ground_truth[i];
        squaredResiduals[i] = residual.array() * residual.array();
    }

    Eigen::VectorXd mean(vectorSize);
    for (int i = 0; i < vectorSize; i++) {
        mean[i] = 0.0;
    }

    for (const auto & squaredResidual : squaredResiduals) {
        mean = mean + squaredResidual;
    }

    mean /= squaredResiduals.size();

    return mean.array().sqrt();
}

Eigen::MatrixXd Tools::calculateJacobian(const Eigen::VectorXd &x_state) {
    return Eigen::MatrixXd();
}
