//
// Created by tri on 24/09/2017.
//

#ifndef BKMAP_OPTION_MANAGER_H
#define BKMAP_OPTION_MANAGER_H

#include <memory>

#include <boost/program_options.hpp>

#include "base/feature_extraction.h"
#include "base/feature_matching.h"
#include "controllers/incremental_mapper.h"
#include "mvs/fusion.h"
#include "mvs/meshing.h"
#include "mvs/patch_match.h"
#include "optim/bundle_adjustment.h"
#include "ui/render_options.h"

namespace bkmap {

    class OptionManager {
    public:
        OptionManager();

        // Create "optimal" set of options for different reconstruction scenarios.
        void InitForIndividualData();
        void InitForVideoData();
        void InitForInternetData();

        void AddAllOptions();
        void AddLogOptions();
        void AddDatabaseOptions();
        void AddImageOptions();
        void AddExtractionOptions();
        void AddMatchingOptions();
        void AddExhaustiveMatchingOptions();
        void AddSequentialMatchingOptions();
        void AddVocabTreeMatchingOptions();
        void AddSpatialMatchingOptions();
        void AddTransitiveMatchingOptions();
        void AddBundleAdjustmentOptions();
        void AddMapperOptions();
        void AddDenseStereoOptions();
        void AddDenseFusionOptions();
        void AddDenseMeshingOptions();
        void AddRenderOptions();

        template <typename T>
        void AddRequiredOption(const std::string& name, T* option,
                               const std::string& help_text = "");
        template <typename T>
        void AddDefaultOption(const std::string& name, T* option,
                              const std::string& help_text = "");

        void Reset();
        bool Check();

        void Parse(const int argc, char** argv);
        bool Read(const std::string& path);
        bool ReRead(const std::string& path);
        void Write(const std::string& path) const;

        std::shared_ptr<std::string> project_path;
        std::shared_ptr<std::string> database_path;
        std::shared_ptr<std::string> image_path;

        std::shared_ptr<ImageReader::Options> image_reader;
        std::shared_ptr<SiftExtractionOptions> sift_extraction;

        std::shared_ptr<SiftMatchingOptions> sift_matching;
        std::shared_ptr<ExhaustiveFeatureMatcher::Options> exhaustive_matching;
        std::shared_ptr<SequentialFeatureMatcher::Options> sequential_matching;
        std::shared_ptr<VocabTreeFeatureMatcher::Options> vocab_tree_matching;
        std::shared_ptr<SpatialFeatureMatcher::Options> spatial_matching;
        std::shared_ptr<TransitiveFeatureMatcher::Options> transitive_matching;

        std::shared_ptr<BundleAdjuster::Options> bundle_adjustment;
        std::shared_ptr<IncrementalMapperController::Options> mapper;

        std::shared_ptr<mvs::PatchMatch::Options> dense_stereo;
        std::shared_ptr<mvs::StereoFusion::Options> dense_fusion;
        std::shared_ptr<mvs::PoissonReconstructionOptions> dense_meshing;

        std::shared_ptr<RenderOptions> render;

    private:
        template <typename T>
        void AddAndRegisterRequiredOption(const std::string& name, T* option,
                                          const std::string& help_text = "");
        template <typename T>
        void AddAndRegisterDefaultOption(const std::string& name, T* option,
                                         const std::string& help_text = "");

        template <typename T>
        void RegisterOption(const std::string& name, const T* option);

        std::shared_ptr<boost::program_options::options_description> desc_;

        std::vector<std::pair<std::string, const bool*>> options_bool_;
        std::vector<std::pair<std::string, const int*>> options_int_;
        std::vector<std::pair<std::string, const double*>> options_double_;
        std::vector<std::pair<std::string, const std::string*>> options_string_;

        bool added_log_options_;
        bool added_database_options_;
        bool added_image_options_;
        bool added_extraction_options_;
        bool added_match_options_;
        bool added_exhaustive_match_options_;
        bool added_sequential_match_options_;
        bool added_vocab_tree_match_options_;
        bool added_spatial_match_options_;
        bool added_transitive_match_options_;
        bool added_ba_options_;
        bool added_mapper_options_;
        bool added_dense_stereo_options_;
        bool added_dense_fusion_options_;
        bool added_dense_meshing_options_;
        bool added_render_options_;
    };

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

    template <typename T>
    void OptionManager::AddRequiredOption(const std::string& name, T* option,
                                          const std::string& help_text) {
        desc_->add_options()(name.c_str(),
                             boost::program_options::value<T>(option)->required(),
                             help_text.c_str());
    }

    template <typename T>
    void OptionManager::AddDefaultOption(const std::string& name, T* option,
                                         const std::string& help_text) {
        desc_->add_options()(
                name.c_str(),
                boost::program_options::value<T>(option)->default_value(*option),
                help_text.c_str());
    }

    template <typename T>
    void OptionManager::AddAndRegisterRequiredOption(const std::string& name,
                                                     T* option,
                                                     const std::string& help_text) {
        desc_->add_options()(name.c_str(),
                             boost::program_options::value<T>(option)->required(),
                             help_text.c_str());
        RegisterOption(name, option);
    }

    template <typename T>
    void OptionManager::AddAndRegisterDefaultOption(const std::string& name,
                                                    T* option,
                                                    const std::string& help_text) {
        desc_->add_options()(
                name.c_str(),
                boost::program_options::value<T>(option)->default_value(*option),
                help_text.c_str());
        RegisterOption(name, option);
    }

    template <typename T>
    void OptionManager::RegisterOption(const std::string& name, const T* option) {
        if (std::is_same<T, bool>::value) {
            options_bool_.emplace_back(name, reinterpret_cast<const bool*>(option));
        } else if (std::is_same<T, int>::value) {
            options_int_.emplace_back(name, reinterpret_cast<const int*>(option));
        } else if (std::is_same<T, double>::value) {
            options_double_.emplace_back(name, reinterpret_cast<const double*>(option));
        } else if (std::is_same<T, std::string>::value) {
            options_string_.emplace_back(name,
                                         reinterpret_cast<const std::string*>(option));
        } else {
//            LOG(FATAL) << "Unsupported option type";
        }
    }

}

#endif //BKMAP_OPTION_MANAGER_H
