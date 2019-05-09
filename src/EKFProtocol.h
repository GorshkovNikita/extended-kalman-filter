//
// Created by nikita on 08.05.19.
//

#ifndef EXTENDED_KALMAN_FILTER_EKFPROTOCOL_H
#define EXTENDED_KALMAN_FILTER_EKFPROTOCOL_H

#include <string_view>

class EKFProtocol {
public:
    std::string_view processMessage(std::string_view message);

private:

};


#endif //EXTENDED_KALMAN_FILTER_EKFPROTOCOL_H
