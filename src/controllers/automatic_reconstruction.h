//
// Created by tri on 25/09/2017.
//

#ifndef BKMAP_AUTOMATIC_RECONSTRUCTION_H
#define BKMAP_AUTOMATIC_RECONSTRUCTION_H

#include <string>

#include "base/reconstruction_manager.h"
#include "util/option_manager.h"
#include "util/threading.h"

namespace bkmap {

    class AutomaticReconstructionController : public Thread {
    public:
        enum class DataType { INDIVIDUAL, VIDEO, INTERNET };
        enum class Quality { LOW, MEDIUM, HIGH };

        struct Options {
            // The path to the workspace folder in which all results are stored.
            std::string workspace_path;

            // The path to the image folder which are used as input.
            std::string image_path;

            // The path to the vocabulary tree for feature matching.
            std::string vocab_tree_path;

            // The type of input data used to choose optimal mapper settings.
            DataType data_type = DataType::INDIVIDUAL;

            // Whether to perform low- or high-quality reconstruction.
            Quality quality = Quality::HIGH;

            // Whether to use shared intrinsics or not.
            bool single_camera = false;

            // Whether to perform sparse mapping.
            bool sparse = true;

// Whether to perform dense mapping.
#ifdef CUDA_ENABLED
            bool dense = true;
#else
            bool dense = false;
#endif

            // The number of threads to use in all stages.
            int num_threads = -1;

            // Whether to use the GPU in feature extraction and matching.
            bool use_gpu = true;

            // Index of the GPU used for GPU stages. For multi-GPU computation,
            // you should separate multiple GPU indices by comma, e.g., "0,1,2,3".
            // By default, all GPUs will be used in all stages.
            std::string gpu_index = "-1";
        };

        AutomaticReconstructionController(
                const Options& options, ReconstructionManager* reconstruction_manager);

        void Stop() override;

    private:
        void Run() override;
        void RunFeatureExtraction();
        void RunFeatureMatching();
        void RunSparseMapper();
        void RunDenseMapper();

        const Options options_;
        OptionManager option_manager_;
        ReconstructionManager* reconstruction_manager_;
        Thread* active_thread_;
        std::unique_ptr<Thread> feature_extractor_;
        std::unique_ptr<Thread> exhaustive_matcher_;
        std::unique_ptr<Thread> sequential_matcher_;
        std::unique_ptr<Thread> vocab_tree_matcher_;
    };

}

#endif //BKMAP_AUTOMATIC_RECONSTRUCTION_H
