//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_MODEL_H
#define BKMAP_MODEL_H

#include <cstdint>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "mvs/depth_map.h"
#include "mvs/image.h"
#include "mvs/normal_map.h"

namespace bkmap {
    namespace mvs {

// Simple sparse model class.
        struct Model {
            struct Point {
                float x = 0;
                float y = 0;
                float z = 0;
                std::vector<int> track;
            };

            // Read the model from different data formats.
            void Read(const std::string& path, const std::string& format);
            void ReadFromCOLMAP(const std::string& path);
            void ReadFromPMVS(const std::string& path);

            // Get the image identifier for the given image name.
            int GetImageId(const std::string& name) const;
            std::string GetImageName(const int image_id) const;

            // For each image, determine the maximally overlapping images, sorted based on
            // the number of shared points subject to a minimum robust average
            // triangulation angle of the points.
            std::vector<std::vector<int>> GetMaxOverlappingImages(
                    const size_t num_images, const double min_triangulation_angle) const;

            // Compute the robust minimum and maximum depths from the sparse point cloud.
            std::vector<std::pair<float, float>> ComputeDepthRanges() const;

            // Compute the number of shared points between all overlapping images.
            std::vector<std::map<int, int>> ComputeSharedPoints() const;

            // Compute the median triangulation angles between all overlapping images.
            std::vector<std::map<int, float>> ComputeTriangulationAngles(
                    const float percentile = 50) const;

            std::vector<Image> images;
            std::vector<Point> points;

        private:
            std::vector<std::string> image_names_;
            std::unordered_map<std::string, int> image_name_to_id_;
        };

    }  // namespace mvs
}

#endif //BKMAP_MODEL_H
