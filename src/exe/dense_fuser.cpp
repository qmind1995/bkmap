//
// Created by tri on 25/09/2017.
//


#include "mvs/fusion.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char* argv[]) {
  InitializeGlog(argv);

  std::string workspace_path;
  std::string input_type = "geometric";
  std::string workspace_format = "COLMAP";
  std::string pmvs_option_name = "option-all";
  std::string output_path;

  OptionManager options;
  options.AddRequiredOption("workspace_path", &workspace_path);
  options.AddDefaultOption("workspace_format", &workspace_format,
                           "{COLMAP, PMVS}");
  options.AddDefaultOption("pmvs_option_name", &pmvs_option_name);
  options.AddDefaultOption("input_type", &input_type,
                           "{photometric, geometric}");
  options.AddRequiredOption("output_path", &output_path);
  options.AddDenseFusionOptions();
  options.Parse(argc, argv);

  StringToLower(&workspace_format);
  if (workspace_format != "bkmap" && workspace_format != "pmvs") {
    std::cout << "ERROR: Invalid `workspace_format` - supported values are "
                 "'COLMAP' or 'PMVS'."
              << std::endl;
    return EXIT_FAILURE;
  }

  StringToLower(&input_type);
  if (input_type != "photometric" && input_type != "geometric") {
    std::cout << "ERROR: Invalid input type - supported values are "
                 "'photometric' and 'geometric'."
              << std::endl;
    return EXIT_FAILURE;
  }

  mvs::StereoFusion fuser(*options.dense_fusion, workspace_path,
                          workspace_format, pmvs_option_name, input_type);

  fuser.Start();
  fuser.Wait();

  std::cout << "Writing output: " << output_path << std::endl;
  WritePlyBinary(output_path, fuser.GetFusedPoints());

  return EXIT_SUCCESS;
}
