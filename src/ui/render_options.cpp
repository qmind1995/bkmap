//
// Created by tri on 26/09/2017.
//

#include "ui/render_options.h"

#include "util/logging.h"

namespace bkmap {

    bool RenderOptions::Check() const {
        CHECK_OPTION_GE(min_track_len, 0);
        CHECK_OPTION_GE(max_error, 0);
        CHECK_OPTION_GT(refresh_rate, 0);
        CHECK_OPTION(projection_type == ProjectionType::PERSPECTIVE ||
                     projection_type == ProjectionType::ORTHOGRAPHIC);
        return true;
    }

}