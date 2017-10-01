//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_UTILS_H
#define BKMAP_UTILS_H

#include <array>
#include <cmath>

namespace bkmap {
    namespace retrieval {

        struct ImageScore {
            int image_id = -1;
            float score = 0.0f;
        };

// Implements the weighting function used to derive a voting weight from the
// Hamming distance of two binary signatures. See Eqn. 4 in
// Arandjelovic, Zisserman. DisLocation: Scalable descriptor distinctiveness for
// location recognition. ACCV 2014.
// The template is the length of the Hamming embedding vectors.
// This class is based on an original implementation by Torsten Sattler.
        template <int N, int kSigma = 16>
        class HammingDistWeightFunctor {
        public:
            HammingDistWeightFunctor() {
                // Fills the look-up table.
                const float sigma_squared = kSigma * kSigma;
                const float max_hamming_dist = 1.5f * kSigma;
                for (int n = 0; n <= N; ++n) {
                    const float hamming_dist = static_cast<float>(n);
                    if (hamming_dist <= max_hamming_dist) {
                        look_up_table_.at(n) =
                                std::exp(-hamming_dist * hamming_dist / sigma_squared);
                    } else {
                        look_up_table_.at(n) = 0.0f;
                    }
                }
            }

            // Returns the weight for Hamming distance h and standard deviation sigma.
            // Does not perform a range check when performing the look-up.
            inline float operator()(const size_t hamming_dist) const {
                return look_up_table_.at(hamming_dist);
            }

        private:
            // In order to avoid wasting computations, we once compute a look-up table
            // storing all function values for all possible values of the standard
            // deviation \sigma. This is implemented as a (N + 1) vector.
            std::array<float, N + 1> look_up_table_;
        };

    }  // namespace retrieval
}

#endif //BKMAP_UTILS_H
