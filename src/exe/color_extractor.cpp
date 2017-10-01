//
// Created by tri on 25/09/2017.
//

#include "base/reconstruction.h"
#include "util/logging.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
  InitializeGlog(argv);

  std::string import_path;
  std::string export_path;

  OptionManager options;
  options.AddImageOptions();
  options.AddDefaultOption("import_path", &import_path);
  options.AddRequiredOption("export_path", &export_path);
  options.Parse(argc, argv);

  Reconstruction reconstruction;
  reconstruction.Read(import_path);
  reconstruction.ExtractColorsForAllImages(*options.image_path);
  reconstruction.Write(export_path);

  return EXIT_SUCCESS;
}