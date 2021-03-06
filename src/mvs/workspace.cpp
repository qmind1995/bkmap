//
// Created by tri on 26/09/2017.
//

#include "mvs/workspace.h"

#include "util/misc.h"

namespace bkmap {
    namespace mvs {

        Workspace::CachedImage::CachedImage() {}

        Workspace::CachedImage::CachedImage(CachedImage&& other) {
            num_bytes = other.num_bytes;
            bitmap = std::move(other.bitmap);
            depth_map = std::move(other.depth_map);
            normal_map = std::move(other.normal_map);
        }

        Workspace::CachedImage& Workspace::CachedImage::operator=(CachedImage&& other) {
            if (this != &other) {
                num_bytes = other.num_bytes;
                bitmap = std::move(other.bitmap);
                depth_map = std::move(other.depth_map);
                normal_map = std::move(other.normal_map);
            }
            return *this;
        }

        size_t Workspace::CachedImage::NumBytes() const { return num_bytes; }

        Workspace::Workspace(const Options& options)
                : options_(options),
                  cache_(1024 * 1024 * 1024 * options_.cache_size,
                         [](const int image_id) { return CachedImage(); }) {
            StringToLower(&options_.input_type);
            model_.Read(options_.workspace_path, options_.workspace_format);
            if (options_.max_image_size > 0) {
                for (auto& image : model_.images) {
                    image.Downsize(options_.max_image_size, options_.max_image_size);
                }
            }

            depth_map_path_ = EnsureTrailingSlash(JoinPaths(
                    options_.workspace_path, options_.stereo_folder, "depth_maps"));
            normal_map_path_ = EnsureTrailingSlash(JoinPaths(
                    options_.workspace_path, options_.stereo_folder, "normal_maps"));
        }

        void Workspace::ClearCache() { cache_.Clear(); }

        const Workspace::Options& Workspace::GetOptions() const {
            return options_;
        }

        const Model& Workspace::GetModel() const { return model_; }

        const Bitmap& Workspace::GetBitmap(const int image_id) {
            auto& cached_image = cache_.GetMutable(image_id);
            if (!cached_image.bitmap) {
                cached_image.bitmap.reset(new Bitmap());
                cached_image.bitmap->Read(GetBitmapPath(image_id), options_.image_as_rgb);
                if (options_.max_image_size > 0) {
                    cached_image.bitmap->Rescale(model_.images.at(image_id).GetWidth(),
                                                 model_.images.at(image_id).GetHeight());
                }
                cached_image.num_bytes += cached_image.bitmap->NumBytes();
                cache_.UpdateNumBytes(image_id);
            }
            return *cached_image.bitmap;
        }

        const DepthMap& Workspace::GetDepthMap(const int image_id) {
            auto& cached_image = cache_.GetMutable(image_id);
            if (!cached_image.depth_map) {
                cached_image.depth_map.reset(new DepthMap());
                cached_image.depth_map->Read(GetDepthMapPath(image_id));
                if (options_.max_image_size > 0) {
                    cached_image.depth_map->Downsize(model_.images.at(image_id).GetWidth(),
                                                     model_.images.at(image_id).GetHeight());
                }
                cached_image.num_bytes += cached_image.depth_map->GetNumBytes();
                cache_.UpdateNumBytes(image_id);
            }
            return *cached_image.depth_map;
        }

        const NormalMap& Workspace::GetNormalMap(const int image_id) {
            auto& cached_image = cache_.GetMutable(image_id);
            if (!cached_image.normal_map) {
                cached_image.normal_map.reset(new NormalMap());
                cached_image.normal_map->Read(GetNormalMapPath(image_id));
                if (options_.max_image_size > 0) {
                    cached_image.normal_map->Downsize(model_.images.at(image_id).GetWidth(),
                                                      model_.images.at(image_id).GetHeight());
                }
                cached_image.num_bytes += cached_image.normal_map->GetNumBytes();
                cache_.UpdateNumBytes(image_id);
            }
            return *cached_image.normal_map;
        }

        std::string Workspace::GetBitmapPath(const int image_id) const {
            return model_.images.at(image_id).GetPath();
        }

        std::string Workspace::GetDepthMapPath(const int image_id) const {
            return depth_map_path_ + GetFileName(image_id);
        }

        std::string Workspace::GetNormalMapPath(const int image_id) const {
            return normal_map_path_ + GetFileName(image_id);
        }

        bool Workspace::HasBitmap(const int image_id) const {
            return ExistsFile(GetBitmapPath(image_id));
        }

        bool Workspace::HasDepthMap(const int image_id) const {
            return ExistsFile(GetDepthMapPath(image_id));
        }

        bool Workspace::HasNormalMap(const int image_id) const {
            return ExistsFile(GetNormalMapPath(image_id));
        }

        std::string Workspace::GetFileName(const int image_id) const {
            const auto& image_name = model_.GetImageName(image_id);
            return StringPrintf("%s.%s.bin", image_name.c_str(),
                                options_.input_type.c_str());
        }

        void ImportPMVSWorkspace(const Workspace& workspace,
                                 const std::string& option_name) {
            const std::string& workspace_path = workspace.GetOptions().workspace_path;
            const std::string& stereo_folder = workspace.GetOptions().stereo_folder;

            CreateDirIfNotExists(JoinPaths(workspace_path, stereo_folder));
            CreateDirIfNotExists(JoinPaths(workspace_path, stereo_folder, "depth_maps"));
            CreateDirIfNotExists(JoinPaths(workspace_path, stereo_folder, "normal_maps"));
            CreateDirIfNotExists(
                    JoinPaths(workspace_path, stereo_folder, "consistency_graphs"));

            const auto option_lines =
                    ReadTextFileLines(JoinPaths(workspace_path, option_name));
            for (const auto& line : option_lines) {
                if (StringStartsWith(line, "timages")) {
                    const auto elems = StringSplit(line, " ");
                    const int num_images = std::stoi(elems[1]);
                    CHECK_EQ(num_images + 2, elems.size());
                    std::vector<std::string> image_names;
                    image_names.reserve(num_images);
                    for (size_t i = 2; i < elems.size(); ++i) {
                        const int image_id = std::stoi(elems[i]);
                        const std::string image_name =
                                workspace.GetModel().GetImageName(image_id);
                        image_names.push_back(image_name);
                    }

                    const auto patch_match_path =
                            JoinPaths(workspace_path, stereo_folder, "patch-match.cfg");
                    const auto fusion_path =
                            JoinPaths(workspace_path, stereo_folder, "fusion.cfg");
                    std::ofstream patch_match_file(patch_match_path, std::ios::trunc);
                    std::ofstream fusion_file(fusion_path, std::ios::trunc);
                    CHECK(patch_match_file.is_open()) << patch_match_path;
                    CHECK(fusion_file.is_open()) << fusion_path;
                    for (const auto ref_image_name : image_names) {
                        patch_match_file << ref_image_name << std::endl;
                        patch_match_file << "__auto__, 20" << std::endl;
                        fusion_file << ref_image_name << std::endl;
                    }
                }
            }
        }

    }  // namespace mvs
}