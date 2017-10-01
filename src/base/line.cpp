//
// Created by tri on 24/09/2017.
//

#include "base/line.h"

#include "util/logging.h"

extern "C" {
#include "ext/LSD/lsd.h"
}

namespace bkmap {

    std::vector<LineSegment> DetectLineSegments(const Bitmap& bitmap,
                                                const double min_length) {
        const double min_length_squared = min_length * min_length;

        std::vector<uint8_t> bitmap_data;
        if (bitmap.IsGrey()) {
            bitmap_data = bitmap.ConvertToRowMajorArray();
        } else {
            const Bitmap bitmap_gray = bitmap.CloneAsGrey();
            bitmap_data = bitmap_gray.ConvertToRowMajorArray();
        }

        std::vector<double> bitmap_data_double(bitmap_data.begin(),
                                               bitmap_data.end());

        int num_segments;
        std::unique_ptr<double> segments_data(lsd(&num_segments,
                                                  bitmap_data_double.data(),
                                                  bitmap.Width(), bitmap.Height()));

        std::vector<LineSegment> segments;
        segments.reserve(num_segments);
        for (int i = 0; i < num_segments; ++i) {
            const Eigen::Vector2d start(segments_data.get()[i * 7],
                                        segments_data.get()[i * 7 + 1]);
            const Eigen::Vector2d end(segments_data.get()[i * 7 + 2],
                                      segments_data.get()[i * 7 + 3]);
            if ((start - end).squaredNorm() >= min_length_squared) {
                segments.emplace_back();
                segments.back().start = start;
                segments.back().end = end;
            }
        }

        return segments;
    }

    std::vector<LineSegmentOrientation> ClassifyLineSegmentOrientations(
            const std::vector<LineSegment>& segments, const double tolerance) {
        CHECK_LE(tolerance, 0.5);

        std::vector<LineSegmentOrientation> orientations;
        orientations.reserve(segments.size());

        for (const auto& segment : segments) {
            const Eigen::Vector2d direction =
                    (segment.end - segment.start).normalized();
            if (std::abs(direction.x()) + tolerance > 1) {
                orientations.push_back(LineSegmentOrientation::HORIZONTAL);
            } else if (std::abs(direction.y()) + tolerance > 1) {
                orientations.push_back(LineSegmentOrientation::VERTICAL);
            } else {
                orientations.push_back(LineSegmentOrientation::UNDEFINED);
            }
        }

        return orientations;
    }

}