#include <iostream>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

int main() {
    VectorXd vec(2);
    vec << 1, 2;
    cout << vec << endl;
    return 0;
}