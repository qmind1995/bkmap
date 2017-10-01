//
// Created by tri on 25/09/2017.
//

#ifndef BKMAP_EUCLIDEAN_TRANSFORM_H
#define BKMAP_EUCLIDEAN_TRANSFORM_H

#include "base/similarity_transform.h"
#include "util/alignment.h"

namespace bkmap {

// N-D Euclidean transform estimator from corresponding point pairs in the
// source and destination coordinate systems.
//
// This algorithm is based on the following paper:
//
//      S. Umeyama. Least-Squares Estimation of Transformation Parameters
//      Between Two Point Patterns. IEEE Transactions on Pattern Analysis and
//      Machine Intelligence, Volume 13 Issue 4, Page 376-380, 1991.
//      http://www.stanford.edu/class/cs273/refs/umeyama.pdf
//
// and uses the Eigen implementation.
    template <int kDim>
    using EuclideanTransformEstimator = SimilarityTransformEstimator<kDim, false>;

}

#endif //BKMAP_EUCLIDEAN_TRANSFORM_H
