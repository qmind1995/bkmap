//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_NORMAL_MAP_H
#define BKMAP_NORMAL_MAP_H

#include <string>
#include <vector>

#include "mvs/mat.h"
#include "util/bitmap.h"

namespace bkmap {
    namespace mvs {

// Normal map class that stores per-pixel normals as a MxNx3 image.
        class NormalMap : public Mat<float> {
        public:
            NormalMap();
            NormalMap(const size_t width, const size_t height);
            explicit NormalMap(const Mat<float>& mat);

            void Rescale(const float factor);
            void Downsize(const size_t max_width, const size_t max_height);

            Bitmap ToBitmap() const;
        };

    }  // namespace mvs
}

#endif //BKMAP_NORMAL_MAP_H
