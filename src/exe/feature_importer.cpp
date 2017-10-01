//
// Created by tri on 25/09/2017.
//

#include "base/camera_models.h"
#include "base/feature_extraction.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

    std::string import_path;
    std::string image_list_path;

    OptionManager options;
    options.AddDatabaseOptions();
    options.AddImageOptions();
    options.AddRequiredOption("import_path", &import_path);
    options.AddDefaultOption("image_list_path", &image_list_path);
    options.AddExtractionOptions();
    options.Parse(argc, argv);

    ImageReader::Options reader_options = *options.image_reader;
    reader_options.database_path = *options.database_path;
    reader_options.image_path = *options.image_path;

    if (!image_list_path.empty()) {
        reader_options.image_list = ReadTextFileLines(image_list_path);
    }

    const std::vector<double> camera_params =
            CSVToVector<double>(options.image_reader->camera_params);
    const int camera_model_id =
            CameraModelNameToId(options.image_reader->camera_model);

    if (camera_params.size() > 0 &&
        !CameraModelVerifyParams(camera_model_id, camera_params)) {
        std::cerr << "ERROR: Invalid camera parameters" << std::endl;
        return EXIT_FAILURE;
    }

    FeatureImporter feature_importer(reader_options, import_path);
    feature_importer.Start();
    feature_importer.Wait();

    return EXIT_SUCCESS;
}