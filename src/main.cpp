#include <iostream>
#include "Eigen/Dense"
#include "json.hpp"
#include "App.h"

using namespace std;
using namespace Eigen;
using namespace nlohmann;

struct SocketData {
    char* data;
};

uWS::TemplatedApp<false>::WebSocketBehavior createWSBehaviour() {
    return uWS::TemplatedApp<false>::WebSocketBehavior {
            .open = [](auto *ws, auto *req) {
                cout << "connected" << endl;
            },
            .message = [](uWS::WebSocket<false, true> *ws, std::string_view message, uWS::OpCode opCode) {
                cout << message << endl;
                ws->send(message, opCode);
            }
    };
}

int main() {
/*
    VectorXd vec(2);
    vec << 1, 2;
    cout << vec << endl;
    json j = {
        {"a", 1}
    };
    cout << j.dump() << endl;
 */
    int port = 4567;
    uWS::App().ws<SocketData>("/*", createWSBehaviour()).listen(port, [port](auto *token) {
        if (token) {
            cout << "Listening on port " << port << endl;
        }
    }).run();
    return 0;
}