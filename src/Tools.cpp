//
// Created by nikita on 09.05.19.
//

#include "Tools.h"
#include <iostream>

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
    Eigen::MatrixXd Hj(3,4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = c1 * c2;

    if (fabs(c1) < 0.0001) {
        std::cout << "CalculateJacobian() - Error - Division by Zero" << std::endl;
        return Hj;
    }

    Hj <<   (px/c2),                (py/c2),                0,      0,
            -(py/c1),               (px/c1),                0,      0,
            py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3,  px/c2,  py/c2;

    return Hj;
}
