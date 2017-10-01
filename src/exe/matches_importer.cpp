//
// Created by tri on 25/09/2017.
//

#include <QApplication>

#include "base/feature_matching.h"
#include "util/logging.h"
#include "util/option_manager.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

#ifdef CUDA_ENABLED
    const bool kUseOpenGL = false;
#else
    const bool kUseOpenGL = true;
#endif

    std::string match_list_path;
    std::string match_type = "pairs";

    OptionManager options;
    options.AddDatabaseOptions();
    options.AddRequiredOption("match_list_path", &match_list_path);
    options.AddDefaultOption("match_type", &match_type,
                             "{'pairs', 'raw', 'inliers'}");
    options.AddMatchingOptions();
    options.Parse(argc, argv);

    std::unique_ptr<QApplication> app;
    if (options.sift_matching->use_gpu && kUseOpenGL) {
        app.reset(new QApplication(argc, argv));
    }

    std::unique_ptr<Thread> feature_matcher;
    if (match_type == "pairs") {
        ImagePairsFeatureMatcher::Options matcher_options;
        matcher_options.match_list_path = match_list_path;
        feature_matcher.reset(new ImagePairsFeatureMatcher(
                matcher_options, *options.sift_matching, *options.database_path));
    } else if (match_type == "raw" || match_type == "inliers") {
        FeaturePairsFeatureMatcher::Options matcher_options;
        matcher_options.match_list_path = match_list_path;
        matcher_options.verify_matches = match_type == "raw";
        feature_matcher.reset(new FeaturePairsFeatureMatcher(
                matcher_options, *options.sift_matching, *options.database_path));
    } else {
        std::cerr << "ERROR: Invalid `match_type`";
        return EXIT_FAILURE;
    }

    if (options.sift_matching->use_gpu && kUseOpenGL) {
        RunThreadWithOpenGLContext(feature_matcher.get());
    } else {
        feature_matcher->Start();
        feature_matcher->Wait();
    }

    return EXIT_SUCCESS;
}