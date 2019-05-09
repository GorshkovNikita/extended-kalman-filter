//
// Created by nikita on 08.05.19.
//

#include "EKFProtocol.h"
#include "json.hpp"
#include "Tools.h"

using namespace nlohmann;

std::string EKFProtocol::processMessage(std::string_view message) {
    if (message.size() > 2 && message[0] == '4' && message[1] == '2') {
        std::string jsonPayload = extractPayload(message.data());
        if (!jsonPayload.empty()) {
            auto payload = json::parse(jsonPayload);
            auto event = payload[0].get<std::string>();
            if (event == "telemetry") {
                std::string sensorMeasurement = payload[1]["sensor_measurement"];
                std::istringstream iss(sensorMeasurement);
                Measurement measurement = parseMeasurement(iss);
                parseGroundTruth(iss);
                return estimate(measurement);
            }
        } else {
            return "42[\"manual\",{}]";
        }
    }
    return "";
}

std::string EKFProtocol::extractPayload(const std::string& message) {
    auto found_null = message.find("null");
    auto b1 = message.find_first_of('[');
    auto b2 = message.find_first_of(']');
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return message.substr(b1, b2 - b1 + 1);
    }
    return "";
}

Measurement EKFProtocol::parseMeasurement(std::istringstream& measurementStream) {
    Measurement measurement;

    long long timestamp;

    std::string sensor_type;
    measurementStream >> sensor_type;

    if (sensor_type == "L") {
        measurement.sensor_type = Measurement::LASER;
        measurement.raw_measurements = Eigen::VectorXd(2);
        float px;
        float py;
        measurementStream >> px;
        measurementStream >> py;
        measurement.raw_measurements << px, py;
        measurementStream >> timestamp;
        measurement.timestamp = timestamp;
    } else if (sensor_type == "R") {
        measurement.sensor_type = Measurement::RADAR;
        measurement.raw_measurements = Eigen::VectorXd(3);
        float ro;
        float theta;
        float ro_dot;
        measurementStream >> ro;
        measurementStream >> theta;
        measurementStream >> ro_dot;
        measurement.raw_measurements << ro,theta, ro_dot;
        measurementStream >> timestamp;
        measurement.timestamp = timestamp;
    }
    return measurement;
}

void EKFProtocol::parseGroundTruth(std::istringstream &measurementStream) {
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    measurementStream >> x_gt;
    measurementStream >> y_gt;
    measurementStream >> vx_gt;
    measurementStream >> vy_gt;

    Eigen::VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt;
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    ground_truth.push_back(gt_values);
}

std::string EKFProtocol::estimate(Measurement& measurement) {
//    fusionEKF.ProcessMeasurement(meas_package);
    Eigen::VectorXd estimate(4);

    double p_x = 0.0; // fusionEKF.ekf_.x_(0);
    double p_y = 0.0; // fusionEKF.ekf_.x_(1);
    double v1  = 0.0; // fusionEKF.ekf_.x_(2);
    double v2 = 0.0; // fusionEKF.ekf_.x_(3);

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;

    estimations.push_back(estimate);

    Eigen::VectorXd rmse = Tools::calculateRMSE(estimations, ground_truth);

    json msgJson;
    msgJson["estimate_x"] = p_x;
    msgJson["estimate_y"] = p_y;
    msgJson["rmse_x"] =  rmse(0);
    msgJson["rmse_y"] =  rmse(1);
    msgJson["rmse_vx"] = rmse(2);
    msgJson["rmse_vy"] = rmse(3);
    return "42[\"estimate_marker\"," + msgJson.dump() + "]";
}
