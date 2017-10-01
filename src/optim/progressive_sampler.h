//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_PROGRESSIVE_SAMPLER_H
#define BKMAP_PROGRESSIVE_SAMPLER_H

#include "optim/sampler.h"

namespace bkmap {

// Random sampler for PROSAC (Progressive Sample Consensus), as described in:
//
//    "Matching with PROSAC - Progressive Sample Consensus".
//        Ondrej Chum and Matas, CVPR 2005.
//
// Note that a separate sampler should be instantiated per thread and that the
// data to be sampled from is assumed to be sorted according to the quality
// function in descending order, i.e., higher quality data is closer to the
// front of the list.
    class ProgressiveSampler : public Sampler {
    public:
        explicit ProgressiveSampler(const size_t num_samples);

        void Initialize(const size_t total_num_samples) override;

        size_t MaxNumSamples() override;

        std::vector<size_t> Sample() override;

    private:
        const size_t num_samples_;
        size_t total_num_samples_;

        // The number of generated samples, i.e. the number of calls to `Sample`.
        size_t t_;
        size_t n_;

        // Variables defined in equation 3.
        double T_n_;
        double T_n_p_;
    };

}

#endif //BKMAP_PROGRESSIVE_SAMPLER_H
