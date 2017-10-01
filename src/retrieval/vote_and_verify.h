//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_VOTE_AND_VERIFY_H
#define BKMAP_VOTE_AND_VERIFY_H

#include "retrieval/geometry.h"

namespace bkmap {
    namespace retrieval {

        struct VoteAndVerifyOptions {
            // Number of top transformations to generate.
            int num_transformations = 10;

            // Number of voting bins in the translation dimension.
            int num_trans_bins = 32;

            // Number of voting bins in the scale dimension.
            int num_scale_bins = 32;

            // Number of voting bins in the orientation dimension.
            int num_angle_bins = 8;

            // Maximum image dimension that bounds the range of the translation bins.
            int max_image_size = 4096;

            // Minimum number of votes for a transformation to be considered.
            int min_num_votes = 1;

            // RANSAC confidence level used to abort the iteration.
            double confidence = 0.99;

            // Thresholds for considering a match an inlier.
            double max_transfer_error = 100.0 * 100.0;
            double max_scale_error = 2.0;
        };

// Compute effective inlier count using Vote-and-Verify by estimating an affine
// transformation from 2D-2D image matches. The method is described in:
//      "A Vote­-and­-Verify Strategy for
//       Fast Spatial Verification in Image Retrieval",
//      Schönberger et al., ACCV 2016.
        int VoteAndVerify(const VoteAndVerifyOptions& options,
                          const std::vector<FeatureGeometryMatch>& matches);

    }  // namespace retrieval
}

#endif //BKMAP_VOTE_AND_VERIFY_H
