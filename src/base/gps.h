//
// Created by tri on 24/09/2017.
//

#ifndef BKMAP_GPS_H
#define BKMAP_GPS_H

#include <vector>

#include <Eigen/Core>

#include "util/alignment.h"
#include "util/types.h"

namespace bkmap {

// Transform ellipsoidal GPS coordinates to Cartesian GPS coordinate
// representation and vice versa.
    class GPSTransform {
    public:
        enum ELLIPSOID { GRS80, WGS84 };

        explicit GPSTransform(const int ellipsoid = GRS80);

        std::vector<Eigen::Vector3d> EllToXYZ(
                const std::vector<Eigen::Vector3d>& ell) const;

        std::vector<Eigen::Vector3d> XYZToEll(
                const std::vector<Eigen::Vector3d>& xyz) const;

    private:
        // Semimajor axis.
        double a_;
        // Semiminor axis.
        double b_;
        // Flattening.
        double f_;
        // Numerical eccentricity.
        double e2_;
    };

}

#endif //BKMAP_GPS_H
