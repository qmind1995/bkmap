//
// Created by tri on 16/09/2017.
//

#ifndef BKMAP_FEATURE_H
#define BKMAP_FEATURE_H

#include <vector>

#include <Eigen/Core>

#include "util/alignment.h"
#include "util/types.h"

namespace bkmap{
    struct FeatureKeypoint {
        // Location of the feature, with the origin at the upper left image corner,
        // i.e. the upper left pixel has the coordinate (0.5, 0.5).
        float x = 0.0f;
        float y = 0.0f;

        // Shape of the feature.
        float scale = 0.0f;
        float orientation = 0.0f;
    };

    struct FeatureMatch {
        // Feature index in first image.
        point2D_t point2D_idx1 = kInvalidPoint2DIdx;

        // Feature index in second image.
        point2D_t point2D_idx2 = kInvalidPoint2DIdx;
    };


    typedef std::vector<FeatureKeypoint> FeatureKeypoints;
    typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            FeatureDescriptors;
    typedef std::vector<FeatureMatch> FeatureMatches;

    // Convert feature keypoints to vector of points.
    std::vector<Eigen::Vector2d> FeatureKeypointsToPointsVector(
            const FeatureKeypoints& keypoints);

    // L2-normalize feature descriptor, where each row represents one feature.
    Eigen::MatrixXf L2NormalizeFeatureDescriptors(
            const Eigen::MatrixXf& descriptors);

    // L1-Root-normalize feature descriptors, where each row represents one feature.
    // See "Three things everyone should know to improve object retrieval",
    // Relja Arandjelovic and Andrew Zisserman, CVPR 2012.
    Eigen::MatrixXf L1RootNormalizeFeatureDescriptors(
            const Eigen::MatrixXf& descriptors);

    // Convert normalized floating point feature descriptor to unsigned byte
    // representation by linear scaling from range [0, 0.5] to [0, 255]. Truncation
    // to a maximum value of 0.5 is used to avoid precision loss and follows the
    // common practice of representing SIFT vectors.
    FeatureDescriptors FeatureDescriptorsToUnsignedByte(
            const Eigen::MatrixXf& descriptors);

// Extract the descriptors corresponding to the largest-scale features.
    void ExtractTopScaleFeatures(FeatureKeypoints* keypoints,
                                 FeatureDescriptors* descriptors,
                                 const size_t num_features);

}

#endif //BKMAP_FEATURE_H
