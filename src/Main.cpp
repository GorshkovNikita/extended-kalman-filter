#include <iostream>
#include "Eigen/Dense"
#include "json.hpp"
#include "App.h"
#include "EKFProtocol.h"
#include "Tools.h"

using namespace std;
using namespace Eigen;
using namespace nlohmann;

struct SocketData {};

int main() {
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // the input list of estimations
    VectorXd e(4);
    e << 1, 1, 0.2, 0.1;
    estimations.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimations.push_back(e);
    e << 3, 3, 0.4, 0.3;
    estimations.push_back(e);

    // the corresponding list of ground truth values
    VectorXd g(4);
    g << 1.1, 1.1, 0.3, 0.2;
    ground_truth.push_back(g);
    g << 2.1, 2.1, 0.4, 0.3;
    ground_truth.push_back(g);
    g << 3.1, 3.1, 0.5, 0.4;
    ground_truth.push_back(g);

    cout << Tools::calculateRMSE(estimations, ground_truth) << endl;
//    int port = 4567;
//    EKFProtocol ekfProtocol;
//    uWS::App()
//        .ws<SocketData>("/*", uWS::TemplatedApp<false>::WebSocketBehavior {
//                .open = [](auto *ws, auto *req) {
//                    cout << "Connected!" << endl;
//                },
//                .message = [&ekfProtocol](auto *ws, std::string_view message, uWS::OpCode opCode) {
//                    cout << message << endl;
//                    ws->send(ekfProtocol.processMessage(message), opCode);
//                }
//        })
//        .listen(port, [port](auto *token) {
//            if (token) {
//                cout << "Listening on port " << port << endl;
//            }
//        })
//        .run();
    return 0;
}