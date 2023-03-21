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
    Reader.read("/home/qzj/code/Wahba-Problem/example_data/bun_zipper_res3.ply", src_cloud);

    int N = src_cloud.size();
    Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
    for (size_t i = 0; i < N; ++i) {
        src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
    }
    std::cout << "The size of src is " << src.rows() << "x" << src.cols() << std::endl;

    Eigen::Matrix3d R = random_rotation_matrix();
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = R * src;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    Eigen::Matrix3d R_quest = shuster::quest(src, tgt);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::cout << "Error = " << getAngularError(R, R_quest) << " deg" << std::endl;
    std::cout << "time = " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << " ms" << std::endl;

    Eigen::Matrix3d R_pseudo = shuster::pseduoSolveRot(src, tgt);
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    std::cout << "Error = " << getAngularError(R, R_pseudo) << " deg" << std::endl;
    std::cout << "time = " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1000.0 << " ms" << std::endl;
    return 0;
}
