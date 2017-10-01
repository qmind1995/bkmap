//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_COMBINATION_SAMPLER_H
#define BKMAP_COMBINATION_SAMPLER_H

#include "optim/sampler.h"

namespace bkmap {

// Random sampler for RANSAC-based methods that generates unique samples.
//
// Note that a separate sampler should be instantiated per thread and it assumes
// that the input data is shuffled in advance.
    class CombinationSampler : public Sampler {
    public:
        explicit CombinationSampler(const size_t num_samples);

        void Initialize(const size_t total_num_samples) override;

        size_t MaxNumSamples() override;

        std::vector<size_t> Sample() override;

    private:
        const size_t num_samples_;
        std::vector<size_t> total_sample_idxs_;
    };

}

#endif //BKMAP_COMBINATION_SAMPLER_H
