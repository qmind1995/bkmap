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

    OptionManager options;
    options.AddDatabaseOptions();
    options.AddExhaustiveMatchingOptions();
    options.Parse(argc, argv);

    std::unique_ptr<QApplication> app;
    if (options.sift_matching->use_gpu && kUseOpenGL) {
        app.reset(new QApplication(argc, argv));
    }

    ExhaustiveFeatureMatcher feature_matcher(*options.exhaustive_matching,
                                             *options.sift_matching,
                                             *options.database_path);

    if (options.sift_matching->use_gpu && kUseOpenGL) {
        RunThreadWithOpenGLContext(&feature_matcher);
    } else {
        feature_matcher.Start();
        feature_matcher.Wait();
    }

    return EXIT_SUCCESS;
}
