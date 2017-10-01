//
// Created by tri on 24/09/2017.
//

#include "base/similarity_transform.h"

#include "base/pose.h"
#include "base/projection.h"
#include "estimators/similarity_transform.h"

namespace bkmap {

    SimilarityTransform3::SimilarityTransform3() {
        SimilarityTransform3(1, ComposeIdentityQuaternion(),
                             Eigen::Vector3d(0, 0, 0));
    }

    SimilarityTransform3::SimilarityTransform3(const Eigen::Matrix3x4d& matrix) {
        transform_.matrix().topLeftCorner<3, 4>() = matrix;
    }

    SimilarityTransform3::SimilarityTransform3(
            const Eigen::Transform<double, 3, Eigen::Affine>& transform)
            : transform_(transform) {}

    SimilarityTransform3::SimilarityTransform3(const double scale,
                                               const Eigen::Vector4d& qvec,
                                               const Eigen::Vector3d& tvec) {
        Eigen::Matrix4d matrix = Eigen::MatrixXd::Identity(4, 4);
        matrix.topLeftCorner<3, 4>() = ComposeProjectionMatrix(qvec, tvec);
        matrix.block<3, 3>(0, 0) *= scale;
        transform_.matrix() = matrix;
    }

    void SimilarityTransform3::Estimate(const std::vector<Eigen::Vector3d>& src,
                                        const std::vector<Eigen::Vector3d>& dst) {
        transform_.matrix().topLeftCorner<3, 4>() =
                SimilarityTransformEstimator<3>().Estimate(src, dst)[0];
    }

    SimilarityTransform3 SimilarityTransform3::Inverse() const {
        return SimilarityTransform3(transform_.inverse());
    }

    void SimilarityTransform3::TransformPoint(Eigen::Vector3d* xyz) const {
        *xyz = transform_ * *xyz;
    }

    void SimilarityTransform3::TransformPose(Eigen::Vector4d* qvec,
                                             Eigen::Vector3d* tvec) const {
        // Projection matrix P1 projects 3D object points to image plane and thus to
        // 2D image points in the source coordinate system:
        //    x' = P1 * X1
        // 3D object points can be transformed to the destination system by applying
        // the similarity transformation S:
        //    X2 = S * X1
        // To obtain the projection matrix P2 that transforms the object point in the
        // destination system to the 2D image points, which do not change:
        //    x' = P2 * X2 = P2 * S * X1 = P1 * S^-1 * S * X1 = P1 * I * X1
        // and thus:
        //    P2' = P1 * S^-1
        // Finally, undo the inverse scaling of the rotation matrix:
        //    P2 = s * P2'

        Eigen::Matrix4d src_matrix = Eigen::MatrixXd::Identity(4, 4);
        src_matrix.topLeftCorner<3, 4>() = ComposeProjectionMatrix(*qvec, *tvec);
        Eigen::Matrix4d dst_matrix =
                src_matrix.matrix() * transform_.inverse().matrix();
        dst_matrix *= Scale();

        *qvec = RotationMatrixToQuaternion(dst_matrix.block<3, 3>(0, 0));
        *tvec = dst_matrix.block<3, 1>(0, 3);
    }

    Eigen::Matrix4d SimilarityTransform3::Matrix() const {
        return transform_.matrix();
    }

    double SimilarityTransform3::Scale() const {
        return Matrix().block<1, 3>(0, 0).norm();
    }

    Eigen::Vector4d SimilarityTransform3::Rotation() const {
        return RotationMatrixToQuaternion(Matrix().block<3, 3>(0, 0) / Scale());
    }

    Eigen::Vector3d SimilarityTransform3::Translation() const {
        return Matrix().block<3, 1>(0, 3);
    }

}