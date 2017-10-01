//
// Created by tri on 25/09/2017.
//

#ifndef BKMAP_HIERARCHICAL_MAPPER_H
#define BKMAP_HIERARCHICAL_MAPPER_H
#include "base/reconstruction_manager.h"
#include "base/scene_clustering.h"
#include "controllers/incremental_mapper.h"
#include "util/threading.h"

namespace bkmap {

// Hierarchical mapping first hierarchically partitions the scene into multiple
// overlapping clusters, then reconstructs them separately using incremental
// mapping, and finally merges them all into a globally consistent
// reconstruction. This is especially useful for larger-scale scenes, since
// incremental mapping becomes slow with an increasing number of images.
    class HierarchicalMapperController : public Thread {
    public:
        struct Options {
            // The path to the image folder which are used as input.
            std::string image_path;

            // The path to the database file which is used as input.
            std::string database_path;

            // The maximum number of trials to initialize a cluster.
            int init_num_trials = 10;

            // The number of workers used to reconstruct clusters in parallel.
            int num_workers = -1;

            bool Check() const;
        };

        HierarchicalMapperController(
                const Options& options,
                const SceneClustering::Options& clustering_options,
                const IncrementalMapperController::Options& mapper_options,
                ReconstructionManager* reconstruction_manager);

    private:
        void Run() override;

        const Options options_;
        const SceneClustering::Options clustering_options_;
        const IncrementalMapperController::Options mapper_options_;
        ReconstructionManager* reconstruction_manager_;
    };

}

#endif //BKMAP_HIERARCHICAL_MAPPER_H
