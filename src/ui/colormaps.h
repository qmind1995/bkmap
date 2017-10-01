//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_COLORMAPS_H
#define BKMAP_COLORMAPS_H

#include <Eigen/Core>

#include "base/reconstruction.h"
#include "util/alignment.h"
#include "util/types.h"

namespace bkmap {

// Base class for 3D point color mapping.
    class PointColormapBase {
    public:
        PointColormapBase();

        virtual void Prepare(EIGEN_STL_UMAP(camera_t, Camera) & cameras,
                             EIGEN_STL_UMAP(image_t, Image) & images,
                             EIGEN_STL_UMAP(point3D_t, Point3D) & points3D,
                             std::vector<image_t>& reg_image_ids) = 0;

        virtual Eigen::Vector3f ComputeColor(const point3D_t point3D_id,
                                             const Point3D& point3D) = 0;

        void UpdateScale(std::vector<float>* values);
        float AdjustScale(const float gray);

        float scale;
        float min;
        float max;
        float range;
        float min_q;
        float max_q;
    };

// Map color according to RGB value from image.
    class PointColormapPhotometric : public PointColormapBase {
    public:
        void Prepare(EIGEN_STL_UMAP(camera_t, Camera) & cameras,
                     EIGEN_STL_UMAP(image_t, Image) & images,
                     EIGEN_STL_UMAP(point3D_t, Point3D) & points3D,
                     std::vector<image_t>& reg_image_ids);

        Eigen::Vector3f ComputeColor(const point3D_t point3D_id,
                                     const Point3D& point3D);
    };

// Map color according to error.
    class PointColormapError : public PointColormapBase {
    public:
        void Prepare(EIGEN_STL_UMAP(camera_t, Camera) & cameras,
                     EIGEN_STL_UMAP(image_t, Image) & images,
                     EIGEN_STL_UMAP(point3D_t, Point3D) & points3D,
                     std::vector<image_t>& reg_image_ids);

        Eigen::Vector3f ComputeColor(const point3D_t point3D_id,
                                     const Point3D& point3D);
    };

// Map color according to track length.
    class PointColormapTrackLen : public PointColormapBase {
    public:
        void Prepare(EIGEN_STL_UMAP(camera_t, Camera) & cameras,
                     EIGEN_STL_UMAP(image_t, Image) & images,
                     EIGEN_STL_UMAP(point3D_t, Point3D) & points3D,
                     std::vector<image_t>& reg_image_ids);

        Eigen::Vector3f ComputeColor(const point3D_t point3D_id,
                                     const Point3D& point3D);
    };

// Map color according to ground-resolution.
    class PointColormapGroundResolution : public PointColormapBase {
    public:
        void Prepare(EIGEN_STL_UMAP(camera_t, Camera) & cameras,
                     EIGEN_STL_UMAP(image_t, Image) & images,
                     EIGEN_STL_UMAP(point3D_t, Point3D) & points3D,
                     std::vector<image_t>& reg_image_ids);

        Eigen::Vector3f ComputeColor(const point3D_t point3D_id,
                                     const Point3D& point3D);

    private:
        std::unordered_map<point3D_t, float> resolutions_;
    };

}

#endif //BKMAP_COLORMAPS_H
