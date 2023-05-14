#include <iostream>
#include <string>
#include "utils.h"
#include "solutions.h"
#include <pcl/io/ply_io.h>
#include <chrono>
using namespace std;

int main(int argc, char **argv) {

    pcl::PointCloud<pcl::PointXYZ> src_cloud = pcl::PointCloud<pcl::PointXYZ>();
    pcl::PLYReader Reader;
    Reader.read("../example_data/bun_zipper_res3.ply", src_cloud);

    std::cout << "The size of src is " << src_cloud.size() << std::endl;
    Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, src_cloud.size());
    for (size_t i = 0; i < src_cloud.size(); ++i) {
        src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
    }
    int num_points = min(100, (int)src_cloud.size());
    src = RandomSample(src, num_points);

    Eigen::Matrix3d R_gt = random_rotation_matrix();
    Eigen::Vector3d t_gt = Eigen::Vector3d::Random() * 1.0;
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = R_gt * src + t_gt.replicate(1, num_points);

    //construct the translation-invariant measurements
    Eigen::Matrix<double, 3, Eigen::Dynamic> src_centered = src.colwise() - src.rowwise().mean();
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_centered = tgt.colwise() - tgt.rowwise().mean();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    Eigen::Matrix3d R_quest = shuster::quest(src_centered, tgt_centered);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::cout << "Error = " << getAngularError(R_gt, R_quest) << " deg" << std::endl;
    std::cout << "time = " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << " ms" << std::endl;

    Eigen::Matrix3d R_pseudo = procrustes::pseduoSolveRot(src_centered, tgt_centered);
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    std::cout << "Error = " << getAngularError(R_gt, R_pseudo) << " deg" << std::endl;
    std::cout << "time = " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1000.0 << " ms" << std::endl;

    Eigen::Matrix3d R_svd = procrustes::SVDSolveRot(src_centered, tgt_centered);
    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    std::cout << "Error = " << getAngularError(R_gt, R_svd) << " deg" << std::endl;
    std::cout << "time = " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() / 1000.0 << " ms" << std::endl;

    return 0;
}
