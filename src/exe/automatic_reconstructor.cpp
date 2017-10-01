//
// Created by tri on 25/09/2017.
//

#include <QApplication>

#include "controllers/automatic_reconstruction.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

#ifdef CUDA_ENABLED
    const bool kUseOpenGL = false;
#else
    const bool kUseOpenGL = true;
#endif

    AutomaticReconstructionController::Options reconstruction_options;
    std::string data_type = "individual";
    std::string quality = "high";

    OptionManager options;
    options.AddRequiredOption("workspace_path",
                              &reconstruction_options.workspace_path);
    options.AddRequiredOption("image_path", &reconstruction_options.image_path);
    options.AddDefaultOption("vocab_tree_path",
                             &reconstruction_options.vocab_tree_path);
    options.AddDefaultOption("data_type", &data_type,
                             "{individual, video, internet}");
    options.AddDefaultOption("quality", &quality, "{low, medium, high}");
    options.AddDefaultOption("single_camera",
                             &reconstruction_options.single_camera);
    options.AddDefaultOption("sparse", &reconstruction_options.sparse);
    options.AddDefaultOption("dense", &reconstruction_options.dense);
    options.AddDefaultOption("num_threads", &reconstruction_options.num_threads);
    options.AddDefaultOption("use_gpu", &reconstruction_options.use_gpu);
    options.AddDefaultOption("gpu_index", &reconstruction_options.gpu_index);
    options.Parse(argc, argv);

    StringToLower(&data_type);
    if (data_type == "individual") {
        reconstruction_options.data_type =
                AutomaticReconstructionController::DataType::INDIVIDUAL;
    } else if (data_type == "video") {
        reconstruction_options.data_type =
                AutomaticReconstructionController::DataType::VIDEO;
    } else if (data_type == "internet") {
        reconstruction_options.data_type =
                AutomaticReconstructionController::DataType::INTERNET;
    } else {
        LOG(FATAL) << "Invalid data type";
    }

    StringToLower(&quality);
    if (quality == "low") {
        reconstruction_options.quality =
                AutomaticReconstructionController::Quality::LOW;
    } else if (quality == "medium") {
        reconstruction_options.quality =
                AutomaticReconstructionController::Quality::MEDIUM;
    } else if (quality == "high") {
        reconstruction_options.quality =
                AutomaticReconstructionController::Quality::HIGH;
    } else {
        LOG(FATAL) << "Invalid data type";
    }

    ReconstructionManager reconstruction_manager;

    if (reconstruction_options.use_gpu && kUseOpenGL) {
        QApplication app(argc, argv);
        AutomaticReconstructionController controller(reconstruction_options,
                                                     &reconstruction_manager);
        RunThreadWithOpenGLContext(&controller);
    } else {
        AutomaticReconstructionController controller(reconstruction_options,
                                                     &reconstruction_manager);
        controller.Start();
        controller.Wait();
    }

    return EXIT_SUCCESS;
}