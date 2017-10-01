//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_RANDOM_SAMPLER_H
#define BKMAP_RANDOM_SAMPLER_H

#include "optim/sampler.h"

namespace bkmap {

// Random sampler for RANSAC-based methods.
//
// Note that a separate sampler should be instantiated per thread.
    class RandomSampler : public Sampler {
    public:
        explicit RandomSampler(const size_t num_samples);

        void Initialize(const size_t total_num_samples) override;

        size_t MaxNumSamples() override;

        std::vector<size_t> Sample() override;

    private:
        const size_t num_samples_;
        std::vector<size_t> sample_idxs_;
    };

}

#endif //BKMAP_RANDOM_SAMPLER_H
