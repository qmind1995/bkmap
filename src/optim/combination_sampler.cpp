//
// Created by tri on 26/09/2017.
//

#include "optim/combination_sampler.h"

#include <numeric>

#include "util/math.h"
#include "util/random.h"

namespace bkmap {

    CombinationSampler::CombinationSampler(const size_t num_samples)
            : num_samples_(num_samples) {}

    void CombinationSampler::Initialize(const size_t total_num_samples) {
        CHECK_LE(num_samples_, total_num_samples);
        total_sample_idxs_.resize(total_num_samples);
        // Note that the samples must be in increasing order for `NextCombination`.
        std::iota(total_sample_idxs_.begin(), total_sample_idxs_.end(), 0);
    }

    size_t CombinationSampler::MaxNumSamples() {
        return NChooseK(total_sample_idxs_.size(), num_samples_);
    }

    std::vector<size_t> CombinationSampler::Sample() {
        std::vector<size_t> sampled_idxs(num_samples_);
        for (size_t i = 0; i < num_samples_; ++i) {
            sampled_idxs[i] = total_sample_idxs_[i];
        }

        if (!NextCombination(total_sample_idxs_.begin(),
                             total_sample_idxs_.begin() + num_samples_,
                             total_sample_idxs_.end())) {
            // Reached all possible combinations, so reset to original state.
            // Note that the samples must be in increasing order for `NextCombination`.
            std::iota(total_sample_idxs_.begin(), total_sample_idxs_.end(), 0);
        }

        return sampled_idxs;
    }

}
