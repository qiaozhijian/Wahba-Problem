#include <iostream>
#include <string>
#include "utils.h"
#include "solutions.h"
#include <pcl/io/ply_io.h>

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

    Eigen::Matrix3d R = random_rotation_matrix();
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = R * src;

    Eigen::Matrix3d R_quest = shuster::quest(src, tgt);
    std::cout << "R = " << std::endl << R << std::endl;
    std::cout << "R_quest = " << std::endl << R_quest << std::endl;
    return 0;
}
