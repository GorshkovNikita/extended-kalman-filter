#include <iostream>
#include "App.h"
#include "EKFProtocol.h"
#include "Tools.h"

struct SocketData {};

int main() {
    int port = 4567;
    EKFProtocol ekfProtocol;
    uWS::App()
        .ws<SocketData>("/*", uWS::TemplatedApp<false>::WebSocketBehavior {
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
        })
        .listen(port, [port](auto *token) {
            if (token) {
                std::cout << "Listening on port " << port << std::endl;
            }
        })
        .run();
    return 0;
}