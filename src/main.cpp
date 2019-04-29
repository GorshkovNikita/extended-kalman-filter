#include <iostream>
#include "Eigen/Dense"
#include "json.hpp"

using namespace std;
using namespace Eigen;
using namespace nlohmann;

int main() {
    VectorXd vec(2);
    vec << 1, 2;
    cout << vec << endl;
    json j = {
        {"a", 1}
    };
    cout << j.dump() << endl;
    return 0;
}