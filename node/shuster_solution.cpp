#include <iostream>
#include <string>
#include "utils.h"
#include "solutions.h"

using namespace std;

int main(int argc, char **argv) {

    Eigen::Matrix3d R_gt = random_rotation_matrix();
    // Test TRIAD algorithm
    Eigen::Vector3d v1, v2, w1, w2;
    v1 = random_unit_vector();
    v2 = random_unit_vector();
    w1 = R_gt * v1;
    w2 = R_gt * v2;
    Eigen::Matrix3d R_triad = shuster::triad(v1, v2, w1, w2);
    std::cout << "Error of R_triad = " << getAngularError(R_gt, R_triad) << " deg" << std::endl;

    int num_vec = 100;
    Eigen::Matrix3Xd V(3, num_vec);
    for (int i = 0; i < num_vec; ++i) {
        V.col(i) = random_unit_vector();
    }
    Eigen::Matrix3Xd W = R_gt * V;
    double noise_level = 0.01;
    noise_level = 0.0;
    for (int i = 0; i < W.cols(); ++i) {
        W.col(i) = add_noise(W.col(i), noise_level);
    }
    Eigen::Matrix3d R_quest = shuster::quest(V, W);
    std::cout << "Error of R_quest = " << getAngularError(R_gt, R_quest) << " deg" << std::endl;
    return 0;
}
