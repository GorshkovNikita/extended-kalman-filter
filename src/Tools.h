//
// Created by nikita on 09.05.19.
//

#ifndef EXTENDED_KALMAN_FILTER_TOOLS_H
#define EXTENDED_KALMAN_FILTER_TOOLS_H

#include "Eigen/Dense"
#include <vector>

class Tools {
public:

    /**
     * A helper method to calculate RMSE.
     */
    static Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                  const std::vector<Eigen::VectorXd> &ground_truth);

    /**
     * A helper method to calculate Jacobians.
     */
    static Eigen::MatrixXd calculateJacobian(const Eigen::VectorXd &x_state);

    static Eigen::VectorXd convertToPolar(const Eigen::VectorXd &x);
    static Eigen::VectorXd convertToCartesian(const Eigen::VectorXd &z);
    static float convertAngle(float angle);
};


#endif //EXTENDED_KALMAN_FILTER_TOOLS_H
