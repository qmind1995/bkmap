//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_LEAST_ABSOLUTE_DEVIATIONS_H
#define BKMAP_LEAST_ABSOLUTE_DEVIATIONS_H

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "util/logging.h"

namespace bkmap {

    struct LeastAbsoluteDeviationsOptions {
        // Augmented Lagrangian parameter.
        double rho = 1.0;

        // Over-relaxation parameter, typical values are between 1.0 and 1.8.
        double alpha = 1.0;

        // Maximum solver iterations.
        int max_num_iterations = 1000;

        // Absolute and relative solution thresholds, as suggested by Boyd et al.
        double absolute_tolerance = 1e-4;
        double relative_tolerance = 1e-2;
    };

// Least absolute deviations (LAD) fitting via ADMM by solving the problem:
//
//        min || A x - b ||_1
//
// The solution is returned in the vector x and the iterative solver is
// initialized with the given value. This implementation is based on the paper
// "Distributed Optimization and Statistical Learning via the Alternating
// Direction Method of Multipliers" by Boyd et al. and the Matlab implementation
// at https://web.stanford.edu/~boyd/papers/admm/least_abs_deviations/lad.html
    bool SolveLeastAbsoluteDeviations(const LeastAbsoluteDeviationsOptions& options,
                                      const Eigen::SparseMatrix<double>& A,
                                      const Eigen::VectorXd& b, Eigen::VectorXd* x);

}


#endif //BKMAP_LEAST_ABSOLUTE_DEVIATIONS_H
