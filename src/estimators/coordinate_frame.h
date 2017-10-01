//
// Created by tri on 25/09/2017.
//

#ifndef BKMAP_COORDINATE_FRAME_H
#define BKMAP_COORDINATE_FRAME_H

#include <Eigen/Core>

#include "base/reconstruction.h"

namespace bkmap {

    struct CoordinateFrameEstimationOptions {
        // The maximum image size for line detection.
        int max_image_size = 1024;
        // The minimum length of line segments in pixels.
        double min_line_length = 3;
        // The tolerance for classifying lines into horizontal/vertical.
        double line_orientation_tolerance = 0.2;
        // The maximum distance in pixels between lines and the vanishing points.
        double max_line_vp_distance = 0.5;
        // The maximum cosine distance between estimated axes to be inliers.
        double max_axis_distance = 0.05;
    };

// Estimate the coordinate frame of the reconstruction assuming a Manhattan
// world by finding the major vanishing points in each image. This function
// assumes that the majority of images is taken in upright direction, i.e.
// people are standing upright in the image. The orthonormal axes of the
// estimated coordinate frame will be given in the columns of the returned
// matrix. If one axis could not be determined, the respective column will be
// zero. The axes are specified in the world coordinate system in the order
// rightward, downward, forward.
    Eigen::Matrix3d EstimateCoordinateFrame(
            const CoordinateFrameEstimationOptions& options,
            const Reconstruction& reconstruction, const std::string& image_path);

}

#endif //BKMAP_COORDINATE_FRAME_H
