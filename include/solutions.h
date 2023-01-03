//
// Created by qzj on 2023/1/3.
//

#ifndef CMAKE_TEMPLATE_SOLUTIONS_H
#define CMAKE_TEMPLATE_SOLUTIONS_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace shuster{

    Eigen::Matrix3d triad(Eigen::Vector3d v1, Eigen::Vector3d v2,
                          Eigen::Vector3d w1, Eigen::Vector3d w2){
        // Rv=w, only accommodate two vectors
        assert(v1.norm() == 1 && v2.norm() == 1 && w1.norm() == 1 && w2.norm() == 1);
        Eigen::Vector3d r1, r2, r3;
        Eigen::Vector3d s1, s2, s3;
        r1 = v1;
        r2 = v1.cross(v2) / v1.cross(v2).norm();
        r3 = r1.cross(r2) / r1.cross(r2).norm();
        s1 = w1;
        s2 = w1.cross(w2) / w1.cross(w2).norm();
        s3 = s1.cross(s2) / s1.cross(s2).norm();
        Eigen::Matrix3d M_ref, M_obs;
        M_ref << r1, r2, r3;
        M_obs << s1, s2, s3;
        Eigen::Matrix3d R = M_obs * M_ref.transpose();
        return R;
    }

}

#endif //CMAKE_TEMPLATE_SOLUTIONS_H
