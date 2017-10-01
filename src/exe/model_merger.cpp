//
// Created by tri on 25/09/2017.
//

#include "base/reconstruction.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

    std::string input_path1;
    std::string input_path2;
    std::string output_path;
    int min_common_images = 3;

    OptionManager options;
    options.AddRequiredOption("input_path1", &input_path1);
    options.AddRequiredOption("input_path2", &input_path2);
    options.AddRequiredOption("output_path", &output_path);
    options.AddDefaultOption("min_common_images", &min_common_images);
    options.Parse(argc, argv);

    Reconstruction reconstruction1;
    reconstruction1.Read(input_path1);
    PrintHeading2("Reconstruction 1");
    std::cout << StringPrintf("Images: %d", reconstruction1.NumRegImages())
    << std::endl;
    std::cout << StringPrintf("Points: %d", reconstruction1.NumPoints3D())
    << std::endl;

    Reconstruction reconstruction2;
    reconstruction2.Read(input_path2);
    PrintHeading2("Reconstruction 2");
    std::cout << StringPrintf("Images: %d", reconstruction2.NumRegImages())
    << std::endl;
    std::cout << StringPrintf("Points: %d", reconstruction2.NumPoints3D())
    << std::endl;

    PrintHeading2("Merging reconstructions");
    if (reconstruction1.Merge(reconstruction2, min_common_images)) {
        std::cout << "=> Merge succeeded" << std::endl;
        PrintHeading2("Merged reconstruction");
        std::cout << StringPrintf("Images: %d", reconstruction1.NumRegImages())
        << std::endl;
        std::cout << StringPrintf("Points: %d", reconstruction1.NumPoints3D())
        << std::endl;
    } else {
        std::cout << "=> Merge failed" << std::endl;
    }

    reconstruction1.Write(output_path);

    return EXIT_SUCCESS;
}
