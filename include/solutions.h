//
// Created by qzj on 2023/1/3.
//

#ifndef CMAKE_TEMPLATE_SOLUTIONS_H
#define CMAKE_TEMPLATE_SOLUTIONS_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace shuster{

    Eigen::Matrix3d quest(const Eigen::Matrix3Xd &V, const Eigen::Matrix3Xd &W){
        Eigen::Matrix3d R;
        Eigen::Matrix3d B = W * V.transpose();
        double sigma = B.trace();
        Eigen::Matrix3d S = B + B.transpose();
        Eigen::Vector3d Z = Eigen::Vector3d::Zero();
        for (int i = 0; i < V.cols(); ++i) {
            Z += W.col(i).cross(V.col(i));
        }
        Eigen::Matrix4d T;
        T.block<3,3>(0,0) = S - sigma * Eigen::Matrix3d::Identity();
        T.block<3,1>(0,3) = Z;
        T.block<1,3>(3,0) = Z.transpose();
        T(3,3) = sigma;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(T);
        Eigen::Vector4d q = es.eigenvectors().col(3);
        Eigen::Quaterniond Q(q(3), q(0), q(1), q(2));
        R = Q.toRotationMatrix().transpose();
        return R;
    }

    Eigen::Matrix3d pseduoSolveRot(const Eigen::Matrix3Xd& src, const Eigen::Matrix3Xd& dst){
        // tgt = R * src
        // tgt * src^T = R * src * src^T
        // R = tgt * src^T * (src * src^T)^-1
        Eigen::Matrix3d R;
        Eigen::Matrix3d src_cov = src * src.transpose();
        R = dst * src.transpose() * src_cov.inverse();
        return R;
    }

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
