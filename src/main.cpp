#include <iostream>
#include "Eigen/Dense"
#include "json.hpp"
#include "App.h"

using namespace std;
using namespace Eigen;
using namespace nlohmann;

int main() {
//    VectorXd vec(2);
//    vec << 1, 2;
//    cout << vec << endl;
//    json j = {
//        {"a", 1}
//    };
//    cout << j.dump() << endl;
    int port = 4567;
    uWS::App().get("hello", [](auto *res, auto *req) {
        res->writeHeader("Content-Type", "text/html; charset=utf-8")->end("Hello HTTP!");
    }).listen(port, [port](auto *token) {
        if (token) {
            std::cout << "Listening on port " << port << std::endl;
        }
    }).run();
    return 0;
}