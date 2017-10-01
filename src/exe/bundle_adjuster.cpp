//
// Created by tri on 25/09/2017.
//

#include "base/reconstruction.h"
#include "controllers/bundle_adjustment.h"
#include "util/logging.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

    std::string input_path;
    std::string output_path;

    OptionManager options;
    options.AddRequiredOption("input_path", &input_path);
    options.AddRequiredOption("output_path", &output_path);
    options.AddBundleAdjustmentOptions();
    options.Parse(argc, argv);

    Reconstruction reconstruction;
    reconstruction.Read(input_path);

    BundleAdjustmentController ba_controller(options, &reconstruction);
    ba_controller.Start();
    ba_controller.Wait();

    reconstruction.Write(output_path);

    return EXIT_SUCCESS;
}