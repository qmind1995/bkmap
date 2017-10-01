//
// Created by tri on 25/09/2017.
//

#include "base/reconstruction.h"
#include "util/logging.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

    std::string input_path;
    std::string output_path;
    std::string output_type;

    OptionManager options;
    options.AddRequiredOption("input_path", &input_path);
    options.AddRequiredOption("output_path", &output_path);
    options.AddRequiredOption("output_type", &output_type,
                              "{'BIN', 'TXT', 'NVM', 'Bundler', 'VRML', 'PLY'}");
    options.Parse(argc, argv);

    Reconstruction reconstruction;
    reconstruction.Read(input_path);

    StringToLower(&output_type);
    if (output_type == "bin") {
        reconstruction.WriteBinary(output_path);
    } else if (output_type == "txt") {
        reconstruction.WriteText(output_path);
    } else if (output_type == "nvm") {
        reconstruction.ExportNVM(output_path);
    } else if (output_type == "bundler") {
        reconstruction.ExportBundler(output_path + ".bundle.out",
                                     output_path + ".list.txt");
    } else if (output_type == "ply") {
        reconstruction.ExportPLY(output_path);
    } else if (output_type == "vrml") {
        const auto base_path = output_path.substr(0, output_path.find_last_of("."));
        reconstruction.ExportVRML(base_path + ".images.wrl",
                                  base_path + ".points3D.wrl", 1,
                                  Eigen::Vector3d(1, 0, 0));
    } else {
        std::cerr << "ERROR: Invalid `output_type`" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
