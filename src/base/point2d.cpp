//
// Created by tri on 16/09/2017.
//

#include "base/point2d.h"

namespace bkmap {

    Point2D::Point2D()
            : xy_(Eigen::Vector2d::Zero()), point3D_id_(kInvalidPoint3DId) {}

}