//
// Created by tri on 24/09/2017.
//

#include "base/gps.h"

#include "util/math.h"

namespace bkmap {

    GPSTransform::GPSTransform(const int ellipsoid) {
        switch (ellipsoid) {
            case GRS80:
                a_ = 6378137;
                b_ = 6.356752314140356e+06;
                f_ = 0.003352810681182;
                break;
            case WGS84:
                a_ = 6378137;
                b_ = 6.356752314245179e+06;
                f_ = 0.003352810664747;
                break;
            default:
                a_ = std::numeric_limits<double>::quiet_NaN();
                b_ = std::numeric_limits<double>::quiet_NaN();
                f_ = std::numeric_limits<double>::quiet_NaN();
                throw std::invalid_argument("Ellipsoid not defined");
        }

        e2_ = (a_ * a_ - b_ * b_) / (a_ * a_);
    }

    std::vector<Eigen::Vector3d> GPSTransform::EllToXYZ(
            const std::vector<Eigen::Vector3d>& ell) const {
        std::vector<Eigen::Vector3d> xyz(ell.size());

        for (size_t i = 0; i < ell.size(); ++i) {
            const double lat = DegToRad(ell[i](0));
            const double lon = DegToRad(ell[i](1));
            const double alt = ell[i](2);

            const double sin_lat = sin(lat);
            const double sin_lon = sin(lon);
            const double cos_lat = cos(lat);
            const double cos_lon = cos(lon);

            // Normalized radius
            const double N = a_ / sqrt(1 - e2_ * sin_lat * sin_lat);

            xyz[i](0) = (N + alt) * cos_lat * cos_lon;
            xyz[i](1) = (N + alt) * cos_lat * sin_lon;
            xyz[i](2) = (N * (1 - e2_) + alt) * sin_lat;
        }

        return xyz;
    }

    std::vector<Eigen::Vector3d> GPSTransform::XYZToEll(
            const std::vector<Eigen::Vector3d>& xyz) const {
        std::vector<Eigen::Vector3d> ell(xyz.size());

        for (size_t i = 0; i < ell.size(); ++i) {
            const double x = xyz[i](0);
            const double y = xyz[i](1);
            const double z = xyz[i](2);

            const double xx = x * x;
            const double yy = y * y;

            const double kEps = 1e-12;

            // Latitude
            double lat = atan2(z, sqrt(xx + yy));
            double alt;

            for (size_t j = 0; j < 100; ++j) {
                const double sin_lat0 = sin(lat);
                const double N = a_ / sqrt(1 - e2_ * sin_lat0 * sin_lat0);
                alt = sqrt(xx + yy) / cos(lat) - N;
                const double prev_lat = lat;
                lat = atan((z / sqrt(xx + yy)) * 1 / (1 - e2_ * N / (N + alt)));

                if (std::abs(prev_lat - lat) < kEps) {
                    break;
                }
            }

            ell[i](0) = RadToDeg(lat);

            // Longitude
            ell[i](1) = RadToDeg(atan2(y, x));
            // Alt
            ell[i](2) = alt;
        }

        return ell;
    }

}
