//
// Created by tri on 26/09/2017.
//

#include "mvs/depth_map.h"

#include "base/warp.h"
#include "util/math.h"

namespace bkmap {
    namespace mvs {

        DepthMap::DepthMap() : DepthMap(0, 0, -1.0f, -1.0f) {}

        DepthMap::DepthMap(const size_t width, const size_t height,
                           const float depth_min, const float depth_max)
                : Mat<float>(width, height, 1),
                  depth_min_(depth_min),
                  depth_max_(depth_max) {}

        DepthMap::DepthMap(const Mat<float>& mat, const float depth_min,
                           const float depth_max)
                : Mat<float>(mat.GetWidth(), mat.GetHeight(), mat.GetDepth()),
                  depth_min_(depth_min),
                  depth_max_(depth_max) {
            CHECK_EQ(mat.GetDepth(), 1);
            data_ = mat.GetData();
        }

        void DepthMap::Rescale(const float factor) {
            if (width_ * height_ == 0) {
                return;
            }

            const size_t new_width = std::round(width_ * factor);
            const size_t new_height = std::round(height_ * factor);
            std::vector<float> new_data(new_width * new_height);
            DownsampleImage(data_.data(), height_, width_, new_height, new_width,
                            new_data.data());

            data_ = new_data;
            width_ = new_width;
            height_ = new_height;

            data_.shrink_to_fit();
        }

        void DepthMap::Downsize(const size_t max_width, const size_t max_height) {
            if (height_ <= max_height && width_ <= max_width) {
                return;
            }
            const float factor_x = static_cast<float>(max_width) / width_;
            const float factor_y = static_cast<float>(max_height) / height_;
            Rescale(std::min(factor_x, factor_y));
        }

        Bitmap DepthMap::ToBitmap(const float min_percentile,
                                  const float max_percentile) const {
            CHECK_GT(width_, 0);
            CHECK_GT(height_, 0);

            Bitmap bitmap;
            bitmap.Allocate(width_, height_, true);

            std::vector<float> valid_depths;
            valid_depths.reserve(data_.size());
            for (const float depth : data_) {
                if (depth > 0) {
                    valid_depths.push_back(depth);
                }
            }

            const float robust_depth_min = Percentile(valid_depths, min_percentile);
            const float robust_depth_max = Percentile(valid_depths, max_percentile);

            const float robust_depth_range = robust_depth_max - robust_depth_min;
            for (size_t y = 0; y < height_; ++y) {
                for (size_t x = 0; x < width_; ++x) {
                    const float depth = Get(y, x);
                    if (depth > 0) {
                        const float robust_depth =
                                std::max(robust_depth_min, std::min(robust_depth_max, depth));
                        const float gray =
                                (robust_depth - robust_depth_min) / robust_depth_range;
                        const BitmapColor<float> color(255 * JetColormap::Red(gray),
                                                       255 * JetColormap::Green(gray),
                                                       255 * JetColormap::Blue(gray));
                        bitmap.SetPixel(x, y, color.Cast<uint8_t>());
                    } else {
                        bitmap.SetPixel(x, y, BitmapColor<uint8_t>(0, 0, 0));
                    }
                }
            }

            return bitmap;
        }

    }  // namespace mvs
}