//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_WORKSPACE_H
#define BKMAP_WORKSPACE_H

#include "mvs/consistency_graph.h"
#include "mvs/depth_map.h"
#include "mvs/model.h"
#include "mvs/normal_map.h"
#include "util/bitmap.h"
#include "util/cache.h"

namespace bkmap {
    namespace mvs {

        class Workspace {
        public:
            struct Options {
                // The maximum cache size in gigabytes.
                double cache_size = 32.0;

                // Maximum image size in either dimension.
                int max_image_size = -1;

                // Whether to read image as RGB or gray scale.
                bool image_as_rgb = true;

                // Location and type of workspace.
                std::string workspace_path;
                std::string workspace_format;
                std::string input_type;
                std::string stereo_folder = "stereo";
            };

            Workspace(const Options& options);

            void ClearCache();

            const Options& GetOptions() const;

            const Model& GetModel() const;
            const Bitmap& GetBitmap(const int image_id);
            const DepthMap& GetDepthMap(const int image_id);
            const NormalMap& GetNormalMap(const int image_id);

            // Get paths to bitmap, depth map, normal map and consistency graph.
            std::string GetBitmapPath(const int image_id) const;
            std::string GetDepthMapPath(const int image_id) const;
            std::string GetNormalMapPath(const int image_id) const;

            // Return whether bitmap, depth map, normal map, and consistency graph exist.
            bool HasBitmap(const int image_id) const;
            bool HasDepthMap(const int image_id) const;
            bool HasNormalMap(const int image_id) const;

        private:
            std::string GetFileName(const int image_id) const;

            class CachedImage {
            public:
                CachedImage();
                CachedImage(CachedImage&& other);
                CachedImage& operator=(CachedImage&& other);
                size_t NumBytes() const;
                size_t num_bytes = 0;
                std::unique_ptr<Bitmap> bitmap;
                std::unique_ptr<DepthMap> depth_map;
                std::unique_ptr<NormalMap> normal_map;

            private:
                NON_COPYABLE(CachedImage)
            };

            Options options_;
            Model model_;
            MemoryConstrainedLRUCache<int, CachedImage> cache_;
            std::string depth_map_path_;
            std::string normal_map_path_;
        };

// Import a PMVS workspace into the COLMAP workspace format. Only images in the
// provided option file name will be imported and used for reconstruction.
        void ImportPMVSWorkspace(const Workspace& workspace,
                                 const std::string& option_name);

    }  // namespace mvs
}

#endif //BKMAP_WORKSPACE_H
