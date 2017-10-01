//
// Created by tri on 24/09/2017.
//

#ifndef BKMAP_BASE_SIMILARITY_TRANSFORM_H
#define BKMAP_BASE_SIMILARITY_TRANSFORM_H

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "util/alignment.h"
#include "util/types.h"

namespace bkmap {

// 3D similarity transformation with 7 degrees of freedom.
    class SimilarityTransform3 {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SimilarityTransform3();

        explicit SimilarityTransform3(const Eigen::Matrix3x4d& matrix);

        explicit SimilarityTransform3(
                const Eigen::Transform<double, 3, Eigen::Affine>& transform);

        SimilarityTransform3(const double scale, const Eigen::Vector4d& qvec,
                             const Eigen::Vector3d& tvec);

        void Estimate(const std::vector<Eigen::Vector3d>& src,
                      const std::vector<Eigen::Vector3d>& dst);

        SimilarityTransform3 Inverse() const;

        void TransformPoint(Eigen::Vector3d* xyz) const;
        void TransformPose(Eigen::Vector4d* qvec, Eigen::Vector3d* tvec) const;

        Eigen::Matrix4d Matrix() const;
        double Scale() const;
        Eigen::Vector4d Rotation() const;
        Eigen::Vector3d Translation() const;

    private:
        Eigen::Transform<double, 3, Eigen::Affine> transform_;
    };

}

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(bkmap::SimilarityTransform3)

#endif //BKMAP_SIMILARITY_TRANSFORM_H
