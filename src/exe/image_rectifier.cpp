//
// Created by tri on 25/09/2017.
//


#include "base/reconstruction.h"
#include "base/undistortion.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace bkmap;

// Read stereo image pair names from a text file. The text file is expected to
// have one image pair per line, e.g.:
//
//      image_name1.jpg image_name2.jpg
//      image_name3.jpg image_name4.jpg
//      image_name5.jpg image_name6.jpg
//      ...
//
std::vector<std::pair<image_t, image_t>> ReadStereoImagePairs(
        const std::string& path, const Reconstruction& reconstruction) {
    const std::vector<std::string> stereo_pair_lines = ReadTextFileLines(path);

    std::vector<std::pair<image_t, image_t>> stereo_pairs;
    stereo_pairs.reserve(stereo_pair_lines.size());

    for (const auto& line : stereo_pair_lines) {
        const std::vector<std::string> names = StringSplit(line, " ");
        CHECK_EQ(names.size(), 2);

        const Image* image1 = reconstruction.FindImageWithName(names[0]);
        const Image* image2 = reconstruction.FindImageWithName(names[1]);

        CHECK_NOTNULL(image1);
        CHECK_NOTNULL(image2);

        stereo_pairs.emplace_back(image1->ImageId(), image2->ImageId());
    }

    return stereo_pairs;
}

int main(int argc, char** argv) {
    InitializeGlog(argv);

    std::string input_path;
    std::string output_path;
    std::string stereo_pairs_list;

    UndistortCameraOptions undistort_camera_options;

    OptionManager options;
    options.AddImageOptions();
    options.AddRequiredOption("input_path", &input_path);
    options.AddRequiredOption("output_path", &output_path);
    options.AddRequiredOption("stereo_pairs_list", &stereo_pairs_list);
    options.AddDefaultOption("blank_pixels",
                             &undistort_camera_options.blank_pixels);
    options.AddDefaultOption("min_scale", &undistort_camera_options.min_scale);
    options.AddDefaultOption("max_scale", &undistort_camera_options.max_scale);
    options.AddDefaultOption("max_image_size",
                             &undistort_camera_options.max_image_size);
    options.Parse(argc, argv);

    Reconstruction reconstruction;
    reconstruction.Read(input_path);

    const auto stereo_pairs =
            ReadStereoImagePairs(stereo_pairs_list, reconstruction);

    StereoImageRectifier rectifier(undistort_camera_options, reconstruction,
                                   *options.image_path, output_path,
                                   stereo_pairs);
    rectifier.Start();
    rectifier.Wait();

    return EXIT_SUCCESS;
}
