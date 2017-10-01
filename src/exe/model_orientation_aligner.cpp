//
// Created by tri on 25/09/2017.
//

#include "estimators/coordinate_frame.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char* argv[]) {
    InitializeGlog(argv);

    std::string input_path;
    std::string output_path;

    CoordinateFrameEstimationOptions frame_estimation_options;

    OptionManager options;
    options.AddImageOptions();
    options.AddRequiredOption("input_path", &input_path);
    options.AddRequiredOption("output_path", &output_path);
    options.AddDefaultOption("max_image_size",
                             &frame_estimation_options.max_image_size);
    options.Parse(argc, argv);

    Reconstruction reconstruction;
    reconstruction.Read(input_path);

    const Eigen::Matrix3d frame = EstimateCoordinateFrame(
            frame_estimation_options, reconstruction, *options.image_path);

    PrintHeading1("Aligning Reconstruction");

    Eigen::Matrix3d tform;
    if (frame.col(0).nonZeros() == 0) {
        std::cout << "Only aligning vertical axis" << std::endl;
        tform = RotationFromUnitVectors(frame.col(1), Eigen::Vector3d(0, -1, 0));
    } else if (frame.col(1).nonZeros() == 0) {
        tform = RotationFromUnitVectors(frame.col(0), Eigen::Vector3d(1, 0, 0));
        std::cout << "Only aligning horizontal axis" << std::endl;
    } else {
        tform = frame.transpose();
        std::cout << "Aligning horizontal and vertical axes" << std::endl;
    }

    std::cout << "using the rotation matrix:" << std::endl;
    std::cout << tform << std::endl;

    reconstruction.Transform(1, RotationMatrixToQuaternion(tform),
                             Eigen::Vector3d(0, 0, 0));

    std::cout << "Writing aligned reconstruction..." << std::endl;
    reconstruction.Write(output_path);

    return EXIT_SUCCESS;
}