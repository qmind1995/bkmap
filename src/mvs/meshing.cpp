//
// Created by tri on 26/09/2017.
//

#include "mvs/meshing.h"

#include <vector>

#include "ext/PoissonRecon/PoissonRecon.h"
#include "ext/PoissonRecon/SurfaceTrimmer.h"
#include "util/logging.h"

namespace bkmap {
    namespace mvs {

        bool PoissonReconstructionOptions::Check() const {
            CHECK_OPTION_GE(point_weight, 0);
            CHECK_OPTION_GT(depth, 0);
            CHECK_OPTION_GE(color, 0);
            CHECK_OPTION_GE(trim, 0);
            CHECK_OPTION_GE(num_threads, -1);
            CHECK_OPTION_NE(num_threads, 0);
            return true;
        }

        bool PoissonReconstruction(const PoissonReconstructionOptions& options,
                                   const std::string& input_path,
                                   const std::string& output_path) {
            CHECK(options.Check());

            std::vector<std::string> args;

            args.push_back("./binary");

            args.push_back("--in");
            args.push_back(input_path);

            args.push_back("--out");
            args.push_back(output_path);

            args.push_back("--pointWeight");
            args.push_back(std::to_string(options.point_weight));

            args.push_back("--depth");
            args.push_back(std::to_string(options.depth));

            if (options.color > 0) {
                args.push_back("--color");
                args.push_back(std::to_string(options.color));
            }

#ifdef OPENMP_ENABLED
            if (options.num_threads > 0) {
                args.push_back("--threads");
                args.push_back(std::to_string(options.num_threads));
            }
#endif  // OPENMP_ENABLED

            if (options.trim > 0) {
                args.push_back("--density");
            }

            std::vector<const char*> args_cstr;
            args_cstr.reserve(args.size());
            for (const auto& arg : args) {
                args_cstr.push_back(arg.c_str());
            }

            if (PoissonRecon(args_cstr.size(), const_cast<char**>(args_cstr.data())) !=
                EXIT_SUCCESS) {
                return false;
            }

            if (options.trim == 0) {
                return true;
            }

            args.clear();
            args_cstr.clear();

            args.push_back("./binary");

            args.push_back("--in");
            args.push_back(output_path);

            args.push_back("--out");
            args.push_back(output_path);

            args.push_back("--trim");
            args.push_back(std::to_string(options.trim));

            args_cstr.reserve(args.size());
            for (const auto& arg : args) {
                args_cstr.push_back(arg.c_str());
            }

            return SurfaceTrimmer(args_cstr.size(),
                                  const_cast<char**>(args_cstr.data())) == EXIT_SUCCESS;
        }

    }  // namespace mvs
}
