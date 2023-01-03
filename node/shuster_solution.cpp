#include <iostream>
#include <string>
#include "utils.h"
#include "solutions.h"

using namespace std;

int main(int argc, char **argv) {

    // Test TRIAD algorithm
    Eigen::Vector3d v1, v2, w1, w2;
    v1 = random_unit_vector();
    v2 = random_unit_vector();
    Eigen::Matrix3d R = random_rotation_matrix();
    w1 = R * v1;
    w2 = R * v2;
    Eigen::Matrix3d R_triad = shuster::triad(v1, v2, w1, w2);
    std::cout << "R = " << std::endl << R << std::endl;
    std::cout << "R_triad = " << std::endl << R_triad << std::endl;

    return 0;
}
