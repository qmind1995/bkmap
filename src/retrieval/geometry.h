//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_GEOMETRY_H
#define BKMAP_GEOMETRY_H

#include <vector>

#include <Eigen/Core>

namespace bkmap {
    namespace retrieval {

        struct FeatureGeometryTransform {
            float scale = 0.0f;
            float angle = 0.0f;
            float tx = 0.0f;
            float ty = 0.0f;
        };

        struct FeatureGeometry {
            // Compute the similarity that transforms the shape of feature 1 to feature 2.
            static FeatureGeometryTransform TransformFromMatch(
                    const FeatureGeometry& feature1, const FeatureGeometry& feature2);
            static Eigen::Matrix<float, 2, 3> TransformMatrixFromMatch(
                    const FeatureGeometry& feature1, const FeatureGeometry& feature2);

            // Get the approximate area occupied by the feature.
            float GetArea() const;

            // Get the approximate area occupied by the feature after applying an affine
            // transformation to the feature geometry.
            float GetAreaUnderTransform(const Eigen::Matrix2f& A) const;

            float x = 0.0f;
            float y = 0.0f;
            float scale = 0.0f;
            float orientation = 0.0f;
        };

// 1-to-M feature geometry match.
        struct FeatureGeometryMatch {
            FeatureGeometry geometry1;
            std::vector<FeatureGeometry> geometries2;
        };

    }  // namespace retrieval
}

#endif //BKMAP_GEOMETRY_H
