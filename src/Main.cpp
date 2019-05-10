#include <iostream>
#include "App.h"
#include "EKFProtocol.h"
#include "Tools.h"

struct SocketData {};

uWS::TemplatedApp<false>::WebSocketBehavior createBehavior() {
    EKFProtocol ekfProtocol;
    uWS::TemplatedApp<false>::WebSocketBehavior behavior;
    behavior.open = [](auto *ws, auto *req) {
        std::cout << "Connected!" << std::endl;
    };
    behavior.message = [&ekfProtocol](auto *ws, std::string_view message, uWS::OpCode opCode) {
        std::string resultMessage = ekfProtocol.processMessage(message);
        if (!resultMessage.empty()) {
//            std::cout << message << std::endl;
//            std::cout << "result = " << resultMessage << std::endl;
            ws->send(resultMessage, opCode);
        }
    };
    return behavior;
}


/*
    uWS::TemplatedApp<false>::WebSocketBehavior {
                .open = [](auto *ws, auto *req) {
                    std::cout << "Connected!" << std::endl;
                },
                .message = [&ekfProtocol](auto *ws, std::string_view message, uWS::OpCode opCode) {
                    std::string resultMessage = ekfProtocol.processMessage(message);
                    if (!resultMessage.empty()) {
//                        std::cout << message << std::endl;
//                        std::cout << "result = " << resultMessage << std::endl;
                        ws->send(resultMessage, opCode);
                    }
                }
        }
 */

int main() {
    int port = 4567;
    uWS::App()
        .ws<SocketData>("/*", createBehavior())
        .listen(port, [port](auto *token) {
            if (token) {
                std::cout << "Listening on port " << port << std::endl;
            }
        })
        .run();
    return 0;
}