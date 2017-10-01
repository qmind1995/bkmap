//
// Created by tri on 25/09/2017.
//


#include "mvs/meshing.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char* argv[]) {
  InitializeGlog(argv);

  std::string input_path;
  std::string output_path;

  OptionManager options;
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDenseMeshingOptions();
  options.Parse(argc, argv);

  CHECK(mvs::PoissonReconstruction(*options.dense_meshing, input_path,
                                   output_path));

  return EXIT_SUCCESS;
}
