//
// Created by tri on 25/09/2017.
//

#include "controllers/automatic_reconstruction.h"

#include "base/feature_extraction.h"
#include "base/feature_matching.h"
#include "base/undistortion.h"
#include "controllers/incremental_mapper.h"
#include "mvs/fusion.h"
#include "mvs/patch_match.h"
#include "util/misc.h"
#include "util/option_manager.h"

namespace bkmap {

    AutomaticReconstructionController::AutomaticReconstructionController(
            const Options& options, ReconstructionManager* reconstruction_manager)
            : options_(options),
              reconstruction_manager_(reconstruction_manager),
              active_thread_(nullptr) {
        CHECK(ExistsDir(options_.workspace_path));
        CHECK(ExistsDir(options_.image_path));
        CHECK_NOTNULL(reconstruction_manager_);

        option_manager_.AddAllOptions();

        *option_manager_.image_path = options_.image_path;
        *option_manager_.database_path =
                JoinPaths(options_.workspace_path, "database.db");

        if (options_.data_type == DataType::VIDEO) {
            option_manager_.InitForVideoData();
        } else if (options_.data_type == DataType::INDIVIDUAL) {
            option_manager_.InitForIndividualData();
        } else if (options_.data_type == DataType::INTERNET) {
            option_manager_.InitForInternetData();
        } else {
            LOG(FATAL) << "Data type not supported";
        }

        if (options_.quality == Quality::LOW) {
            option_manager_.sift_extraction->max_image_size = 1000;
            option_manager_.sequential_matching->loop_detection_num_images /= 2;
            option_manager_.vocab_tree_matching->num_images /= 2;
            option_manager_.mapper->ba_local_max_num_iterations /= 2;
            option_manager_.mapper->ba_global_max_num_iterations /= 2;
            option_manager_.mapper->ba_global_images_ratio *= 1.2;
            option_manager_.mapper->ba_global_points_ratio *= 1.2;
            option_manager_.mapper->ba_global_max_refinements = 2;
            option_manager_.dense_stereo->max_image_size = 1000;
            option_manager_.dense_stereo->window_radius = 4;
            option_manager_.dense_stereo->num_samples /= 2;
            option_manager_.dense_stereo->num_iterations = 3;
            option_manager_.dense_stereo->geom_consistency = false;
            option_manager_.dense_fusion->check_num_images /= 2;
            option_manager_.dense_fusion->max_image_size = 1000;
        } else if (options_.quality == Quality::MEDIUM) {
            option_manager_.sift_extraction->max_image_size = 1600;
            option_manager_.sequential_matching->loop_detection_num_images /= 1.5;
            option_manager_.vocab_tree_matching->num_images /= 1.5;
            option_manager_.mapper->ba_local_max_num_iterations /= 1.5;
            option_manager_.mapper->ba_global_max_num_iterations /= 1.5;
            option_manager_.mapper->ba_global_images_ratio *= 1.1;
            option_manager_.mapper->ba_global_points_ratio *= 1.1;
            option_manager_.mapper->ba_global_max_refinements = 2;
            option_manager_.dense_stereo->max_image_size = 1600;
            option_manager_.dense_stereo->window_radius = 5;
            option_manager_.dense_stereo->num_samples /= 1.5;
            option_manager_.dense_stereo->num_iterations = 5;
            option_manager_.dense_stereo->geom_consistency = false;
            option_manager_.dense_fusion->check_num_images /= 1.5;
            option_manager_.dense_fusion->max_image_size = 1600;
        }  // else: high quality is the default.

        option_manager_.sift_extraction->num_threads = options_.num_threads;
        option_manager_.sift_matching->num_threads = options_.num_threads;
        option_manager_.mapper->num_threads = options_.num_threads;
        option_manager_.dense_meshing->num_threads = options_.num_threads;

        ImageReader::Options reader_options = *option_manager_.image_reader;
        reader_options.database_path = *option_manager_.database_path;
        reader_options.image_path = *option_manager_.image_path;
        reader_options.single_camera = options_.single_camera;

        option_manager_.sift_extraction->use_gpu = options_.use_gpu;
        option_manager_.sift_matching->use_gpu = options_.use_gpu;

        option_manager_.sift_extraction->gpu_index = options_.gpu_index;
        option_manager_.sift_matching->gpu_index = options_.gpu_index;
        option_manager_.dense_stereo->gpu_index = options_.gpu_index;

        feature_extractor_.reset(new SiftFeatureExtractor(
                reader_options, *option_manager_.sift_extraction));

        exhaustive_matcher_.reset(new ExhaustiveFeatureMatcher(
                *option_manager_.exhaustive_matching, *option_manager_.sift_matching,
                *option_manager_.database_path));

        if (!options_.vocab_tree_path.empty()) {
            option_manager_.sequential_matching->loop_detection = true;
            option_manager_.sequential_matching->vocab_tree_path =
                    options_.vocab_tree_path;
        }

        sequential_matcher_.reset(new SequentialFeatureMatcher(
                *option_manager_.sequential_matching, *option_manager_.sift_matching,
                *option_manager_.database_path));

        if (!options_.vocab_tree_path.empty()) {
            option_manager_.vocab_tree_matching->vocab_tree_path =
                    options_.vocab_tree_path;
            vocab_tree_matcher_.reset(new VocabTreeFeatureMatcher(
                    *option_manager_.vocab_tree_matching, *option_manager_.sift_matching,
                    *option_manager_.database_path));
        }
    }

    void AutomaticReconstructionController::Stop() {
        if (active_thread_ != nullptr) {
            active_thread_->Stop();
        }
        Thread::Stop();
    }

    void AutomaticReconstructionController::Run() {
        if (IsStopped()) {
            return;
        }

        RunFeatureExtraction();

        if (IsStopped()) {
            return;
        }

        RunFeatureMatching();

        if (IsStopped()) {
            return;
        }

        if (options_.sparse) {
            RunSparseMapper();
        }

        if (IsStopped()) {
            return;
        }
    }

    void AutomaticReconstructionController::RunFeatureExtraction() {
        CHECK(feature_extractor_);
        active_thread_ = feature_extractor_.get();
        feature_extractor_->Start();
        feature_extractor_->Wait();
        feature_extractor_.reset();
        active_thread_ = nullptr;
    }

    void AutomaticReconstructionController::RunFeatureMatching() {
        Thread* matcher = nullptr;
        if (options_.data_type == DataType::VIDEO) {
            matcher = sequential_matcher_.get();
        } else if (options_.data_type == DataType::INDIVIDUAL ||
                   options_.data_type == DataType::INTERNET) {
            Database database(*option_manager_.database_path);
            const size_t num_images = database.NumImages();
            if (options_.vocab_tree_path.empty() || num_images < 200) {
                matcher = exhaustive_matcher_.get();
            } else {
                matcher = vocab_tree_matcher_.get();
            }
        }

        CHECK(matcher);
        active_thread_ = matcher;
        matcher->Start();
        matcher->Wait();
        exhaustive_matcher_.reset();
        sequential_matcher_.reset();
        vocab_tree_matcher_.reset();
        active_thread_ = nullptr;
    }

    void AutomaticReconstructionController::RunSparseMapper() {
        const auto sparse_path = JoinPaths(options_.workspace_path, "sparse");
        if (ExistsDir(sparse_path)) {
            auto dir_list = GetDirList(sparse_path);
            std::sort(dir_list.begin(), dir_list.end());
            if (dir_list.size() > 0) {
                std::cout << std::endl
                << "WARNING: Skipping sparse reconstruction because it is "
                        "already computed"
                << std::endl;
                for (const auto& dir : dir_list) {
                    reconstruction_manager_->Read(dir);
                }
                return;
            }
        }

        IncrementalMapperController mapper(
                option_manager_.mapper.get(), *option_manager_.image_path,
                *option_manager_.database_path, reconstruction_manager_);
        active_thread_ = &mapper;
        mapper.Start();
        mapper.Wait();
        active_thread_ = nullptr;

        CreateDirIfNotExists(sparse_path);
        reconstruction_manager_->Write(sparse_path, &option_manager_);
    }

    void AutomaticReconstructionController::RunDenseMapper() {

    }

}