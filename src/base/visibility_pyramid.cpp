//
// Created by tri on 17/09/2017.
//

#include "base/visibility_pyramid.h"
#include "util/logging.h"
#include "util/math.h"

namespace bkmap {

    VisibilityPyramid::VisibilityPyramid() : VisibilityPyramid(0, 0, 0) {}

    VisibilityPyramid::VisibilityPyramid(const size_t num_levels,
                                         const size_t width, const size_t height)
            : width_(width), height_(height), score_(0), max_score_(0) {
        pyramid_.resize(num_levels);
        for (size_t level = 0; level < num_levels; ++level) {
            const size_t level_plus_one = level + 1;
            const int dim = 1 << level_plus_one;
            pyramid_[level].setZero(dim, dim);
            max_score_ += dim * dim * dim * dim;
        }
    }

    void VisibilityPyramid::SetPoint(const double x, const double y) {
        CHECK_GT(pyramid_.size(), 0);

        size_t cx = 0;
        size_t cy = 0;
        CellForPoint(x, y, &cx, &cy);

        for (int i = static_cast<int>(pyramid_.size() - 1); i >= 0; --i) {
            auto& level = pyramid_[i];

            level(cy, cx) += 1;
            if (level(cy, cx) == 1) {
                score_ += level.size();
            }

            cx = cx >> 1;
            cy = cy >> 1;
        }

        CHECK_LE(score_, max_score_);
    }

    void VisibilityPyramid::ResetPoint(const double x, const double y) {
        CHECK_GT(pyramid_.size(), 0);

        size_t cx = 0;
        size_t cy = 0;
        CellForPoint(x, y, &cx, &cy);

        for (int i = static_cast<int>(pyramid_.size() - 1); i >= 0; --i) {
            auto& level = pyramid_[i];

            level(cy, cx) -= 1;
            if (level(cy, cx) == 0) {
                score_ -= level.size();
            }

            cx = cx >> 1;
            cy = cy >> 1;
        }

        CHECK_LE(score_, max_score_);
    }

    void VisibilityPyramid::CellForPoint(const double x, const double y, size_t* cx,
                                         size_t* cy) const {
        CHECK_GT(width_, 0);
        CHECK_GT(height_, 0);
        const int max_dim = 1 << pyramid_.size();
        *cx = Clip<size_t>(static_cast<size_t>(max_dim * x / width_), 0,
                           static_cast<size_t>(max_dim - 1));
        *cy = Clip<size_t>(static_cast<size_t>(max_dim * y / height_), 0,
                           static_cast<size_t>(max_dim - 1));
    }

}