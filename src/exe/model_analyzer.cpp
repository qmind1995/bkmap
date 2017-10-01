//
// Created by tri on 25/09/2017.
//

#include "base/reconstruction.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

    std::string path;

    OptionManager options;
    options.AddRequiredOption("path", &path);
    options.Parse(argc, argv);

    Reconstruction reconstruction;
    reconstruction.Read(path);

    std::cout << StringPrintf("Cameras: %d", reconstruction.NumCameras())
    << std::endl;
    std::cout << StringPrintf("Images: %d", reconstruction.NumImages())
    << std::endl;
    std::cout << StringPrintf("Registered images: %d",
                              reconstruction.NumRegImages())
    << std::endl;
    std::cout << StringPrintf("Points: %d", reconstruction.NumPoints3D())
    << std::endl;
    std::cout << StringPrintf("Observations: %d",
                              reconstruction.ComputeNumObservations())
    << std::endl;
    std::cout << StringPrintf("Mean track length: %f",
                              reconstruction.ComputeMeanTrackLength())
    << std::endl;
    std::cout << StringPrintf("Mean observations per image: %f",
                              reconstruction.ComputeMeanObservationsPerRegImage())
    << std::endl;
    std::cout << StringPrintf("Mean reprojection error: %fpx",
                              reconstruction.ComputeMeanReprojectionError())
    << std::endl;

    return EXIT_SUCCESS;
}