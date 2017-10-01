//
// Created by tri on 25/09/2017.
//

#include "base/database.h"
#include "util/logging.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
  InitializeGlog(argv);

  OptionManager options;
  options.AddDatabaseOptions();
  options.Parse(argc, argv);

  Database database(*options.database_path);

  return EXIT_SUCCESS;
}
