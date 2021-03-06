//
// Created by tri on 24/09/2017.
//

#include "base/projection.h"

#include "base/pose.h"

namespace bkmap {

    Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Vector4d& qvec,
                                              const Eigen::Vector3d& tvec) {
        Eigen::Matrix3x4d proj_matrix;
        proj_matrix.leftCols<3>() = QuaternionToRotationMatrix(qvec);
        proj_matrix.rightCols<1>() = tvec;
        return proj_matrix;
    }

    Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Matrix3d& R,
                                              const Eigen::Vector3d& t) {
        Eigen::Matrix3x4d proj_matrix;
        proj_matrix.leftCols<3>() = R;
        proj_matrix.rightCols<1>() = t;
        return proj_matrix;
    }

    Eigen::Matrix3x4d InvertProjectionMatrix(const Eigen::Matrix3x4d& proj_matrix) {
        Eigen::Matrix3x4d inv_proj_matrix;
        inv_proj_matrix.leftCols<3>() = proj_matrix.leftCols<3>().transpose();
        inv_proj_matrix.rightCols<1>() = ProjectionCenterFromMatrix(proj_matrix);
        return inv_proj_matrix;
    }

    Eigen::Vector2d ProjectPointToImage(const Eigen::Vector3d& point3D,
                                        const Eigen::Matrix3x4d& proj_matrix,
                                        const Camera& camera) {
        const Eigen::Vector3d world_point = proj_matrix * point3D.homogeneous();
        return camera.WorldToImage(world_point.hnormalized());
    }

    double CalculateReprojectionError(const Eigen::Vector2d& point2D,
                                      const Eigen::Vector3d& point3D,
                                      const Eigen::Matrix3x4d& proj_matrix,
                                      const Camera& camera) {
        const auto image_point = ProjectPointToImage(point3D, proj_matrix, camera);
        return (image_point - point2D).norm();
    }

    double CalculateAngularError(const Eigen::Vector2d& point2D,
                                 const Eigen::Vector3d& point3D,
                                 const Eigen::Matrix3x4d& proj_matrix,
                                 const Camera& camera) {
        return CalculateAngularError(camera.ImageToWorld(point2D), point3D,
                                     proj_matrix);
    }

    double CalculateAngularError(const Eigen::Vector2d& point2D,
                                 const Eigen::Vector3d& point3D,
                                 const Eigen::Matrix3x4d& proj_matrix) {
        const Eigen::Vector3d ray1 = point2D.homogeneous();
        const Eigen::Vector3d ray2 = proj_matrix * point3D.homogeneous();
        return std::acos(ray1.normalized().transpose() * ray2.normalized());
    }

    double CalculateDepth(const Eigen::Matrix3x4d& proj_matrix,
                          const Eigen::Vector3d& point3D) {
        const double d = (proj_matrix.row(2) * point3D.homogeneous()).sum();
        return d * proj_matrix.col(2).norm();
    }

    bool HasPointPositiveDepth(const Eigen::Matrix3x4d& proj_matrix,
                               const Eigen::Vector3d& point3D) {
        return (proj_matrix(2, 0) * point3D(0) + proj_matrix(2, 1) * point3D(1) +
                proj_matrix(2, 2) * point3D(2) + proj_matrix(2, 3)) >
               std::numeric_limits<double>::epsilon();
    }

}
