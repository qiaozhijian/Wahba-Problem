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

inline Eigen::Vector3d add_noise(Eigen::Vector3d v, double std) {
    Eigen::Vector3d noise;
    noise << random_normal(std), random_normal(std), random_normal(std);
    return (v + noise).normalized();
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

inline Eigen::Matrix<double, 3, Eigen::Dynamic> RandomSample(Eigen::Matrix<double, 3, Eigen::Dynamic> &src, int sample_num) {
    Eigen::PermutationMatrix<3> perm(3);
    perm.setIdentity();
    std::random_shuffle(perm.indices().data(), perm.indices().data() + perm.indices().size());
    src = perm * src;

    Eigen::Matrix<double, 3, Eigen::Dynamic> src_sample(3, sample_num);
    for (size_t i = 0; i < sample_num; ++i) {
        src_sample.col(i) = src.col(i);
    }
    return src_sample;
}

inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
    return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0))) * 180 / M_PI;
}

#endif //CMAKE_TEMPLATE_UTILS_H
