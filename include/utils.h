//
// Created by qzj on 2023/1/3.
//
#ifndef CMAKE_TEMPLATE_UTILS_H
#define CMAKE_TEMPLATE_UTILS_H
#include <random>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

inline double random_normal(double std) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dis(0, std);
    double number = dis(gen);
    return number;
}

inline Eigen::Vector3d random_unit_vector() {
    Eigen::Vector3d v;
    v << random_normal(1.0), random_normal(1.0), random_normal(1.0);
    v.normalize();
    return v;
}

inline Eigen::Quaterniond random_unit_quaternion() {
    Eigen::Quaterniond q;
    q.w() = random_normal(1);
    q.vec() = random_unit_vector();
    q.normalize();
    return q;
}

inline Eigen::Matrix3d random_rotation_matrix() {
    Eigen::Quaterniond q = random_unit_quaternion();
    Eigen::Matrix3d R = q.toRotationMatrix();
    return R;
}

#endif //CMAKE_TEMPLATE_UTILS_H
