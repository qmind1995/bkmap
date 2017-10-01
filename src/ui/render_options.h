//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_RENDER_OPTIONS_H
#define BKMAP_RENDER_OPTIONS_H

#include <iostream>

namespace bkmap {

    struct RenderOptions {
        enum ProjectionType {
            PERSPECTIVE,
            ORTHOGRAPHIC,
        };

        // Minimum track length for a point to be rendered.
        int min_track_len = 3;

        // Maximum error for a point to be rendered.
        double max_error = 2;

        // The rate of registered images at which to refresh.
        int refresh_rate = 1;

        // Whether to automatically adjust the refresh rate. The bigger the
        // reconstruction gets, the less frequently the scene is rendered.
        bool adapt_refresh_rate = true;

        // Whether to visualize image connections.
        bool image_connections = false;

        // The projection type of the renderer.
        int projection_type = ProjectionType::PERSPECTIVE;

        bool Check() const;
    };

}

#endif //BKMAP_RENDER_OPTIONS_H
