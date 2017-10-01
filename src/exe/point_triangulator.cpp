//
// Created by tri on 25/09/2017.
//

#include "controllers/incremental_mapper.h"
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
    options.AddImageOptions();
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

    const auto& mapper_options = *options.mapper;

    PrintHeading1("Loading database");

    DatabaseCache database_cache;

    {
        Database database(*options.database_path);
        Timer timer;
        timer.Start();
        const size_t min_num_matches =
                static_cast<size_t>(mapper_options.min_num_matches);
        database_cache.Load(database, min_num_matches,
                            mapper_options.ignore_watermarks,
                            mapper_options.image_names);
        std::cout << std::endl;
        timer.PrintMinutes();
    }

    std::cout << std::endl;

    Reconstruction reconstruction;
    reconstruction.Read(import_path);

    CHECK_GE(reconstruction.NumRegImages(), 2)
        << "Need at least two images for triangulation";

    IncrementalMapper mapper(&database_cache);
    mapper.BeginReconstruction(&reconstruction);

    //////////////////////////////////////////////////////////////////////////////
    // Triangulation
    //////////////////////////////////////////////////////////////////////////////

    const auto tri_options = mapper_options.Triangulation();

    for (const image_t image_id : reconstruction.RegImageIds()) {
        const auto& image = reconstruction.Image(image_id);

        PrintHeading1("Triangulating image #" + std::to_string(image_id));

        const size_t num_existing_points3D = image.NumPoints3D();

        std::cout << "  => Image has " << num_existing_points3D << " / "
        << image.NumObservations() << " points" << std::endl;

        mapper.TriangulateImage(tri_options, image_id);

        std::cout << "  => Triangulated "
        << (image.NumPoints3D() - num_existing_points3D) << " points"
        << std::endl;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Bundle adjustment
    //////////////////////////////////////////////////////////////////////////////

    CompleteAndMergeTracks(mapper_options, &mapper);

    const auto ba_options = mapper_options.GlobalBundleAdjustment();

    // Configure bundle adjustment.
    BundleAdjustmentConfig ba_config;
    for (const image_t image_id : reconstruction.RegImageIds()) {
        ba_config.AddImage(image_id);
        ba_config.SetConstantPose(image_id);
        ba_config.SetConstantCamera(reconstruction.Image(image_id).CameraId());
    }

    for (int i = 0; i < mapper_options.ba_global_max_refinements; ++i) {
        // Avoid degeneracies in bundle adjustment.
        reconstruction.FilterObservationsWithNegativeDepth();

        const size_t num_observations = reconstruction.ComputeNumObservations();

        PrintHeading1("Bundle adjustment");
        BundleAdjuster bundle_adjuster(ba_options, ba_config);
        CHECK(bundle_adjuster.Solve(&reconstruction));

        size_t num_changed_observations = 0;
        num_changed_observations += CompleteAndMergeTracks(mapper_options, &mapper);
        num_changed_observations += FilterPoints(mapper_options, &mapper);
        const double changed =
                static_cast<double>(num_changed_observations) / num_observations;
        std::cout << StringPrintf("  => Changed observations: %.6f", changed)
        << std::endl;
        if (changed < mapper_options.ba_global_max_refinement_change) {
            break;
        }
    }

    PrintHeading1("Extracting colors");
    reconstruction.ExtractColorsForAllImages(*options.image_path);

    const bool kDiscardReconstruction = false;
    mapper.EndReconstruction(kDiscardReconstruction);

    reconstruction.Write(export_path);

    return EXIT_SUCCESS;
}