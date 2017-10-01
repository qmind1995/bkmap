//
// Created by tri on 25/09/2017.
//

#include "sfm/incremental_mapper.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

    std::string import_path;
    std::string export_path;

    OptionManager options;
    options.AddDatabaseOptions();
    options.AddRequiredOption("import_path", &import_path);
    options.AddRequiredOption("export_path", &export_path);
    options.AddMapperOptions();
    options.Parse(argc, argv);

    if (!ExistsDir(import_path)) {
        std::cerr << "ERROR: `import_path` is not a directory" << std::endl;
        return EXIT_FAILURE;
    }

    if (!ExistsDir(export_path)) {
        std::cerr << "ERROR: `export_path` is not a directory" << std::endl;
        return EXIT_FAILURE;
    }

    PrintHeading1("Loading database");

    DatabaseCache database_cache;

    {
        Database database(*options.database_path);
        Timer timer;
        timer.Start();
        const size_t min_num_matches =
                static_cast<size_t>(options.mapper->min_num_matches);
        database_cache.Load(database, min_num_matches,
                            options.mapper->ignore_watermarks,
                            options.mapper->image_names);
        std::cout << std::endl;
        timer.PrintMinutes();
    }

    std::cout << std::endl;

    Reconstruction reconstruction;
    reconstruction.Read(import_path);

    IncrementalMapper mapper(&database_cache);
    mapper.BeginReconstruction(&reconstruction);

    const auto mapper_options = options.mapper->Mapper();

    for (const auto& image : reconstruction.Images()) {
        if (image.second.IsRegistered()) {
            continue;
        }

        PrintHeading1("Registering image #" + std::to_string(image.first) + " (" +
                      std::to_string(reconstruction.NumRegImages() + 1) + ")");

        std::cout << "  => Image sees " << image.second.NumVisiblePoints3D()
        << " / " << image.second.NumObservations() << " points"
        << std::endl;

        mapper.RegisterNextImage(mapper_options, image.first);
    }

    const bool kDiscardReconstruction = false;
    mapper.EndReconstruction(kDiscardReconstruction);

    reconstruction.Write(export_path);

    return EXIT_SUCCESS;
}