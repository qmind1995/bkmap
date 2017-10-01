//
// Created by tri on 25/09/2017.
//

#ifndef BKMAP_GENERALIZED_ABSOLUTE_POSE_COEFFS_H
#define BKMAP_GENERALIZED_ABSOLUTE_POSE_COEFFS_H

#include <Eigen/Core>

namespace bkmap {

    Eigen::Matrix<double, 9, 1> ComputeDepthsSylvesterCoeffs(
            const Eigen::Matrix<double, 3, 6>& K);

}

#endif //BKMAP_GENERALIZED_ABSOLUTE_POSE_COEFFS_H
