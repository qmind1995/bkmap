//
// Created by tri on 25/09/2017.
//


#include "base/reconstruction.h"
#include "base/undistortion.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

    std::string input_path;
    std::string output_path;
    std::string output_type = "COLMAP";

    UndistortCameraOptions undistort_camera_options;

    OptionManager options;
    options.AddImageOptions();
    options.AddRequiredOption("input_path", &input_path);
    options.AddRequiredOption("output_path", &output_path);
    options.AddDefaultOption("output_type", &output_type);
    options.AddDefaultOption("blank_pixels",
                             &undistort_camera_options.blank_pixels);
    options.AddDefaultOption("min_scale", &undistort_camera_options.min_scale);
    options.AddDefaultOption("max_scale", &undistort_camera_options.max_scale);
    options.AddDefaultOption("max_image_size",
                             &undistort_camera_options.max_image_size);
    options.Parse(argc, argv);

    CreateDirIfNotExists(output_path);

    Reconstruction reconstruction;
    reconstruction.Read(input_path);

    std::unique_ptr<Thread> undistorter;
    if (output_type == "COLMAP") {
        undistorter.reset(new COLMAPUndistorter(undistort_camera_options,
                                                reconstruction, *options.image_path,
                                                output_path));
    } else if (output_type == "PMVS") {
        undistorter.reset(new PMVSUndistorter(undistort_camera_options,
                                              reconstruction, *options.image_path,
                                              output_path));
    } else if (output_type == "CMP-MVS") {
        undistorter.reset(new CMPMVSUndistorter(undistort_camera_options,
                                                reconstruction, *options.image_path,
                                                output_path));
    } else {
        std::cerr << "ERROR: Invalid `output_type` - supported values are "
                "{'COLMAP', 'PMVS', 'CMP-MVS'}."
        << std::endl;
        return EXIT_FAILURE;
    }

    undistorter->Start();
    undistorter->Wait();

    return EXIT_SUCCESS;
}
