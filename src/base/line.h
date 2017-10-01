//
// Created by tri on 24/09/2017.
//

#ifndef BKMAP_LINE_H
#define BKMAP_LINE_H

#include <Eigen/Core>

#include "util/alignment.h"
#include "util/bitmap.h"

namespace bkmap {

    struct LineSegment {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector2d start;
        Eigen::Vector2d end;
    };

    enum class LineSegmentOrientation {
        HORIZONTAL = 1,
        VERTICAL = -1,
        UNDEFINED = 0,
    };

// Detect line segments in the given bitmap image.
    std::vector<LineSegment> DetectLineSegments(const Bitmap& bitmap,
                                                const double min_length = 3);

// Classify line segments into horizontal/vertical.
    std::vector<LineSegmentOrientation> ClassifyLineSegmentOrientations(
            const std::vector<LineSegment>& segments, const double tolerance = 0.25);

}

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(bkmap::LineSegment)

#endif //BKMAP_LINE_H
