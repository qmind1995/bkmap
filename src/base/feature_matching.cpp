//
// Created by tri on 24/09/2017.
//

#include "base/feature_matching.h"

#include <fstream>
#include <numeric>

#include "base/camera_models.h"
#include "base/database.h"
#include "base/gps.h"
#include "estimators/essential_matrix.h"
#include "estimators/two_view_geometry.h"
#include "ext/SiftGPU/SiftGPU.h"
#include "optim/ransac.h"
#include "retrieval/visual_index.h"
#include "util/cuda.h"
#include "util/misc.h"
#include "ext/FLANN/flann.hpp"
#include "ext/VLFeat/kdtree.h"

namespace bkmap {
    namespace {

        void PrintElapsedTime(const Timer& timer) {
            std::cout << StringPrintf(" in %.3fs", timer.ElapsedSeconds()) << std::endl;
        }

        void IndexImagesInVisualIndex(const int num_threads, const int max_num_features,
                                      const std::vector<image_t>& image_ids,
                                      Thread* thread, FeatureMatcherCache* cache,
                                      retrieval::VisualIndex<>* visual_index) {
            retrieval::VisualIndex<>::IndexOptions index_options;
            index_options.num_threads = num_threads;

            for (size_t i = 0; i < image_ids.size(); ++i) {
                if (thread->IsStopped()) {
                    return;
                }

                Timer timer;
                timer.Start();

                std::cout << StringPrintf("Indexing image [%d/%d]", i + 1, image_ids.size())
                << std::flush;

                auto keypoints = cache->GetKeypoints(image_ids[i]);
                auto descriptors = cache->GetDescriptors(image_ids[i]);
                if (max_num_features > 0 && descriptors.rows() > max_num_features) {
                    ExtractTopScaleFeatures(&keypoints, &descriptors, max_num_features);
                }

                visual_index->Add(index_options, image_ids[i], keypoints, descriptors);

                PrintElapsedTime(timer);
            }

            // Compute the TF-IDF weights, etc.
            visual_index->Prepare();
        }

        void MatchNearestNeighborsInVisualIndex(
                const int num_threads, const int num_images, const int num_verifications,
                const int max_num_features, const std::vector<image_t>& image_ids,
                Thread* thread, FeatureMatcherCache* cache,
                retrieval::VisualIndex<>* visual_index, SiftFeatureMatcher* matcher) {
            struct Retrieval {
                image_t image_id = kInvalidImageId;
                std::vector<retrieval::ImageScore> image_scores;
            };

            // Create a thread pool to retrieve the nearest neighbors.
            ThreadPool retrieval_thread_pool(num_threads);
            JobQueue<Retrieval> retrieval_queue(num_threads);

            // The retrieval thread kernel function. Note that the descriptors should be
            // extracted outside of this function sequentially to avoid any concurrent
            // access to the database causing race conditions.
            retrieval::VisualIndex<>::QueryOptions query_options;
            query_options.max_num_images = num_images;
            query_options.max_num_verifications = num_verifications;
            auto QueryFunc = [&](const image_t image_id) {
                auto keypoints = cache->GetKeypoints(image_id);
                auto descriptors = cache->GetDescriptors(image_id);
                if (max_num_features > 0 && descriptors.rows() > max_num_features) {
                    ExtractTopScaleFeatures(&keypoints, &descriptors, max_num_features);
                }

                Retrieval retrieval;
                retrieval.image_id = image_id;
                visual_index->QueryWithVerification(query_options, keypoints, descriptors,
                                                    &retrieval.image_scores);

                CHECK(retrieval_queue.Push(retrieval));
            };

            // Initially, make all retrieval threads busy and continue with the matching.
            size_t image_idx = 0;
            const size_t init_num_tasks =
                    std::min(image_ids.size(), 2 * retrieval_thread_pool.NumThreads());
            for (; image_idx < init_num_tasks; ++image_idx) {
                retrieval_thread_pool.AddTask(QueryFunc, image_ids[image_idx]);
            }

            std::vector<std::pair<image_t, image_t>> image_pairs;

            // Pop the finished retrieval results and enqueue them for feature matching.
            for (size_t i = 0; i < image_ids.size(); ++i) {
                if (thread->IsStopped()) {
                    retrieval_queue.Stop();
                    return;
                }

                Timer timer;
                timer.Start();

                std::cout << StringPrintf("Matching image [%d/%d]", i + 1, image_ids.size())
                << std::flush;

                // Push the next image to the retrieval queue.
                if (image_idx < image_ids.size()) {
                    retrieval_thread_pool.AddTask(QueryFunc, image_ids[image_idx]);
                    image_idx += 1;
                }

                // Pop the next results from the retrieval queue.
                const auto retrieval = retrieval_queue.Pop();
                CHECK(retrieval.IsValid());

                const auto& image_id = retrieval.Data().image_id;
                const auto& image_scores = retrieval.Data().image_scores;

                // Compose the image pairs from the scores.
                image_pairs.clear();
                image_pairs.reserve(image_scores.size());
                for (const auto image_score : image_scores) {
                    image_pairs.emplace_back(image_id, image_score.image_id);
                }

                matcher->Match(image_pairs);

                PrintElapsedTime(timer);
            }
        }

        Eigen::MatrixXi ComputeSiftCosinDistanceMatrix(
                const FeatureKeypoints* keypoints1, const FeatureKeypoints* keypoints2,
                const FeatureDescriptors& descriptors1,
                const FeatureDescriptors& descriptors2){

            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors1_int =
                    descriptors1.cast<int>();
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors2_int =
                    descriptors2.cast<int>();

            Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists(
                    descriptors1.rows(), descriptors2.rows());

            for (FeatureDescriptors::Index i1 = 0; i1 < descriptors1.rows(); ++i1) {
                for (FeatureDescriptors::Index i2 = 0; i2 < descriptors2.rows(); ++i2) {
                    dists(i1, i2) = descriptors1_int.row(i1).dot(descriptors2_int.row(i2));
                }
            }

            return dists;
        }

        Eigen::MatrixXd ComputeSiftEuclideanDistanceMatrix(
                const FeatureKeypoints* keypoints1, const FeatureKeypoints* keypoints2,
                const FeatureDescriptors& descriptors1,
                const FeatureDescriptors& descriptors2,
                const std::function<bool(float, float, float, float)>& guided_filter) {
            if (guided_filter != nullptr) {
                CHECK_NOTNULL(keypoints1);
                CHECK_NOTNULL(keypoints2);
                CHECK_EQ(keypoints1->size(), descriptors1.rows());
                CHECK_EQ(keypoints2->size(), descriptors2.rows());
            }

            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors1_int =
                    descriptors1.cast<int>();
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors2_int =
                    descriptors2.cast<int>();

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists(
                    descriptors1.rows(), descriptors2.rows());

            for (FeatureDescriptors::Index i1 = 0; i1 < descriptors1.rows(); ++i1) {
                for (FeatureDescriptors::Index i2 = 0; i2 < descriptors2.rows(); ++i2) {
                    if (guided_filter != nullptr &&
                        guided_filter((*keypoints1)[i1].x, (*keypoints1)[i1].y,
                                      (*keypoints2)[i2].x, (*keypoints2)[i2].y)) {
                        dists(i1, i2) = 0;
                    } else {
                        double dist_ = 0;
                        for(int fi =0; fi< 128; fi++){
                            dist_ += ( descriptors1_int(i1, fi) - descriptors2_int(i2, fi) ) * ( descriptors1_int(i1, fi) - descriptors2_int(i2, fi) );
                        }
                        dists(i1, i2) = std::sqrt(dist_);
                    }
                }
            }

            return dists;
        }

        Eigen::MatrixXd ComputeSiftChiSquareDistanceMatrix(const FeatureKeypoints* keypoints1, const FeatureKeypoints* keypoints2,
                                                           const FeatureDescriptors& descriptors1,
                                                           const FeatureDescriptors& descriptors2){
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors1_int =
                    descriptors1.cast<int>();
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors2_int =
                    descriptors2.cast<int>();
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists(
                    descriptors1.rows(), descriptors2.rows());
            for (FeatureDescriptors::Index i1 = 0; i1 < descriptors1.rows(); ++i1) {
                for (FeatureDescriptors::Index i2 = 0; i2 < descriptors2.rows(); ++i2) {
                    double dist_ = 0;
                    for(int fi =0; fi< 128; fi++){
                        if(descriptors1_int(i1, fi) + descriptors2_int(i2, fi) >= 0.01 ){
                            dist_ +=  ( descriptors1_int(i1, fi) - descriptors2_int(i2, fi) )
                                     * ( descriptors1_int(i1, fi) - descriptors2_int(i2, fi) )
                                     / ( descriptors1_int(i1, fi) + descriptors2_int(i2, fi) );
                        }
                    }
                    dists(i1, i2) = dist_;

                }
            }
            return dists;
        }

        Eigen::MatrixXd ComputeSiftPEARSON_CORRELATIONDistanceMatrix(const FeatureKeypoints* keypoints1, const FeatureKeypoints* keypoints2,
                                                                 const FeatureDescriptors& descriptors1,
                                                                 const FeatureDescriptors& descriptors2){
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors1_int =
                    descriptors1.cast<int>();
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors2_int =
                    descriptors2.cast<int>();
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists(
                    descriptors1.rows(), descriptors2.rows());

            for (FeatureDescriptors::Index i1 = 0; i1 < descriptors1.rows(); ++i1) {
                for (FeatureDescriptors::Index i2 = 0; i2 < descriptors2.rows(); ++i2) {
                    double dist_upper = 0;
                    double dist_lower = 0;
                    double dist_avg_1 = 0;
                    double dist_avg_2 = 0;

                    for (int fi = 0; fi < 128; fi++) {
                        dist_avg_1 += descriptors1_int(i1, fi);
                        dist_avg_2 += descriptors1_int(i2, fi);
                    }
                    dist_avg_1 = dist_avg_1/128;
                    dist_avg_2 = dist_avg_2/128;

                    for (int fi = 0; fi < 128; fi++) {
                        double tmp = (descriptors1_int(i1, fi) - dist_avg_1) * (descriptors1_int(i2, fi) - dist_avg_2);
                        dist_upper += tmp;
                        dist_lower += tmp*tmp;
                    }
                    if(dist_lower <= 0.01){
                        dists(i1, i2) = 0;
                    }
                    else{
                        dists(i1, i2) = dist_upper/ std::sqrt(dist_lower);
                    }
                }
            }

            return dists;
        }

        Eigen::MatrixXi ComputeSiftManhattanDistanceMatrix(const FeatureKeypoints* keypoints1, const FeatureKeypoints* keypoints2,
                                                            const FeatureDescriptors& descriptors1,
                                                            const FeatureDescriptors& descriptors2){
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors1_int =
                    descriptors1.cast<int>();
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors2_int =
                    descriptors2.cast<int>();

            Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists(
                    descriptors1.rows(), descriptors2.rows());

            for (FeatureDescriptors::Index i1 = 0; i1 < descriptors1.rows(); ++i1) {
                for (FeatureDescriptors::Index i2 = 0; i2 < descriptors2.rows(); ++i2) {
                    int dist_ = 0;
                    for(int fi =0; fi< 128; fi++){
                        dist_ += std::abs( descriptors1_int(i1, fi) - descriptors2_int(i2, fi) );
                    }
                    dists(i1, i2) = dist_;
                }
            }

            return dists;
        }

        Eigen::MatrixXi ComputeSiftChebyshevDistanceMatrix(const FeatureKeypoints* keypoints1, const FeatureKeypoints* keypoints2,
                                                           const FeatureDescriptors& descriptors1,
                                                           const FeatureDescriptors& descriptors2){
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors1_int =
                    descriptors1.cast<int>();
            const Eigen::Matrix<int, Eigen::Dynamic, 128> descriptors2_int =
                    descriptors2.cast<int>();

            Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists(
                    descriptors1.rows(), descriptors2.rows());

            for (FeatureDescriptors::Index i1 = 0; i1 < descriptors1.rows(); ++i1) {
                for (FeatureDescriptors::Index i2 = 0; i2 < descriptors2.rows(); ++i2) {
                    int dist_ = 0;
                    for(int fi =0; fi< 128; fi++){
                        dist_ += std::max(std::abs( descriptors1_int(i1, fi) - descriptors2_int(i2, fi) ) , 0) ;
                    }
                    dists(i1, i2) = dist_;
                }
            }

            return dists;
        }

        size_t FindBestMatchesOneWay_i(SiftMatchingOptions::FeatureDistance dType,
                                       const Eigen::MatrixXi& dists, const float max_ratio, std::vector<int>* matches){
            const double kDistNorm = 1.0f / (512.0f * 512.0f);
            size_t num_matches = 0;
            matches->resize(dists.rows(), -1);

            int best = 0;
            bool isSmaller_better = true;
            switch (dType){
                case SiftMatchingOptions::FeatureDistance::EUCLIDEAN :
                case SiftMatchingOptions::FeatureDistance::MANHATTAN:
                case SiftMatchingOptions::FeatureDistance::CHI_SQUARE:
                case SiftMatchingOptions::FeatureDistance::PEARSON_CORRELATION:
                case SiftMatchingOptions::FeatureDistance::MINKOWSKI:{
                    best = 100000000;
                    break;
                }
                case SiftMatchingOptions::FeatureDistance::COSINE:{
                    isSmaller_better = false;
                    break;
                }
            }

            for (Eigen::MatrixXd::Index i1 = 0; i1 < dists.rows(); ++i1) {
                int best_i2 = -1;
                int best_dist = best;
                int second_best_dist = best;

                for (Eigen::MatrixXd::Index i2 = 0; i2 < dists.cols(); ++i2) {
                    const int dist = dists(i1, i2);
                    if(isSmaller_better){
                        if (dist < best_dist) {
                            best_i2 = i2;
                            second_best_dist = best_dist;
                            best_dist = dist;
                        } else if (dist < second_best_dist) {
                            second_best_dist = dist;
                        }
                    }
                    else{
                        if (dist > best_dist) {
                            best_i2 = i2;
                            second_best_dist = best_dist;
                            best_dist = dist;
                        } else if(dist < second_best_dist){
                            second_best_dist = dist;
                        }
                    }
                }

                // Check if any match found.
                if (best_i2 == -1 ) {
                    continue;
                    std::cout << StringPrintf("skip this keypoint case 1 \n");
                }

                double best_dist_normed = 0;
                double second_best_dist_normed = 0;

                if(dType == SiftMatchingOptions::FeatureDistance::COSINE){
                    best_dist_normed = std::acos(std::min(kDistNorm * best_dist, 1.0));
                    second_best_dist_normed = std::acos(std::min(kDistNorm * second_best_dist, 1.0));
                }
                else{
                    best_dist_normed = kDistNorm * best_dist;
                    second_best_dist_normed = kDistNorm * second_best_dist;
                }


                // Check if match passes ratio test. Keep this comparison >= in order to
                // ensure that the case of best == second_best is detected.
                if (best_dist_normed >= max_ratio * second_best_dist_normed) {
                    std::cout << StringPrintf("skip this keypoint case 2 \n");
                    continue;
                }

                std::cout << StringPrintf("add this keypoint, distance = %d \n ", best_dist_normed);
                num_matches += 1;
                (*matches)[i1] = best_i2;
            }

            return num_matches;
        }

        size_t FindBestMatchesOneWay_d(SiftMatchingOptions::FeatureDistance dType,
                                       const Eigen::MatrixXd& dists, const float max_ratio, std::vector<int>* matches){
            const double kDistNorm = 1.0f / (512.0f * 512.0f);

            size_t num_matches = 0;
            matches->resize(dists.rows(), -1);
            double best = 0;
            bool isSmaller_better = true;
            switch (dType){
                case SiftMatchingOptions::FeatureDistance::EUCLIDEAN :
                case SiftMatchingOptions::FeatureDistance::MANHATTAN:
                case SiftMatchingOptions::FeatureDistance::CHI_SQUARE:
                case SiftMatchingOptions::FeatureDistance ::PEARSON_CORRELATION:
                case SiftMatchingOptions::FeatureDistance::MINKOWSKI:{
                    best = 100000000;
                    break;
                }
                case SiftMatchingOptions::FeatureDistance::COSINE:{
                    isSmaller_better = false;
                    break;
                }
            }

            for (Eigen::MatrixXd::Index i1 = 0; i1 < dists.rows(); ++i1) {
                int best_i2 = -1;
                double best_dist = best;
                double second_best_dist = best;

                for (Eigen::MatrixXd::Index i2 = 0; i2 < dists.cols(); ++i2) {
                    const double dist = dists(i1, i2);
                    if(isSmaller_better){
                        if (dist < best_dist) {
                            best_i2 = i2;
                            second_best_dist = best_dist;
                            best_dist = dist;
                        } else if (dist < second_best_dist) {
                            second_best_dist = dist;
                        }
                    }
                    else{
                        if (dist > best_dist) {
                            best_i2 = i2;
                            second_best_dist = best_dist;
                            best_dist = dist;
                        } else if(dist < second_best_dist){
                            second_best_dist = dist;
                        }
                    }
                }

                // Check if any match found.
                if (best_i2 == -1 ) {
                    continue;
                    std::cout << StringPrintf("skip this keypoint case 1 \n");
                }

                double best_dist_normed = 0;
                double second_best_dist_normed = 0;

                if(dType == SiftMatchingOptions::FeatureDistance::COSINE){
                    best_dist_normed = std::acos(std::min(kDistNorm * best_dist, 1.0));
                    second_best_dist_normed = std::acos(std::min(kDistNorm * second_best_dist, 1.0));
                }
                else{
                    best_dist_normed = kDistNorm * best_dist;
                    second_best_dist_normed = kDistNorm * second_best_dist;
                }


                // Check if match passes ratio test. Keep this comparison >= in order to
                // ensure that the case of best == second_best is detected.
                if (best_dist_normed >= max_ratio * second_best_dist_normed) {
                    std::cout << StringPrintf("skip this keypoint case 2 \n");
                    continue;
                }

                std::cout << StringPrintf("add this keypoint, distance = %d \n ", best_dist_normed);
                num_matches += 1;
                (*matches)[i1] = best_i2;
            }

            return num_matches;
        }

        size_t FindBestMatchesOneWay(const Eigen::MatrixXi& cosDists,
                                     const Eigen::MatrixXd& dists,
                                     const float max_ratio, const float max_distance,
                                     std::vector<int>* matches) {
            // SIFT descriptor vectors are normalized to length 512.
            const float kDistNorm = 1.0f / (512.0f * 512.0f);

            size_t num_matches = 0;
            matches->resize(dists.rows(), -1);

            for (Eigen::MatrixXd::Index i1 = 0; i1 < dists.rows(); ++i1) {
                int best_i2 = -1;
                int cos_best_i2 = -1;
                double best_dist = 100000000;
                double second_best_dist = 100000000;
                int cos_best_dist = 0;
                int cos_second_best_dist = 0;

                for (Eigen::MatrixXd::Index i2 = 0; i2 < dists.cols(); ++i2) {
                    const double dist = dists(i1, i2);
                    if (dist < best_dist) {
                        best_i2 = i2;
                        second_best_dist = best_dist;
                        best_dist = dist;
                    } else if (dist < second_best_dist) {
                        second_best_dist = dist;
                    }

                    const int cosDist = cosDists(i1, i2);
                    if (cosDist > cos_best_dist) {
                        cos_best_i2 = i2;
                        cos_second_best_dist = cos_best_dist;
                        cos_best_dist = cosDist;
                    } else if(cosDist < cos_second_best_dist){
                        cos_second_best_dist = cosDist;
                    }
                }

                // Check if any match found.
                if (best_i2 == -1 || cos_best_i2 == -1 || best_i2 != cos_best_i2) {
                    continue;
                    std::cout << StringPrintf("skip this keypoint case 1 \n");
                }

                const float cos_best_dist_normed =
                        std::acos(std::min(kDistNorm * cos_best_dist, 1.0f));

                const double best_dist_normed = kDistNorm * best_dist;

                // Check if match distance passes threshold.
//                if (best_dist_normed > max_distance) {
//                    continue;
//                }


                const double second_best_dist_normed = kDistNorm * second_best_dist;
                const float cos_second_best_dist_normed =
                        std::acos(std::min(kDistNorm * cos_second_best_dist, 1.0f));

                // Check if match passes ratio test. Keep this comparison >= in order to
                // ensure that the case of best == second_best is detected.
                if (best_dist_normed >= max_ratio * second_best_dist_normed
                    || cos_best_dist_normed >=  max_ratio * cos_second_best_dist_normed) {
                    std::cout << StringPrintf("skip this keypoint case 2 \n");
                    continue;
                }

                std::cout << StringPrintf("add this keypoint, distance = %d \n ", best_dist_normed);
                num_matches += 1;
                (*matches)[i1] = best_i2;
            }

            return num_matches;
        }

        void FindBestMatches_d(SiftMatchingOptions::FeatureDistance dType, const Eigen::MatrixXd& dists, const double max_ratio,
                               FeatureMatches* matches){
            matches->clear();
            std::vector<int> matches12;
            const size_t num_matches12 = FindBestMatchesOneWay_d(dType, dists, max_ratio, &matches12);
            std::vector<int> matches21;
            const size_t num_matches21 = FindBestMatchesOneWay_d(dType, dists.transpose(), max_ratio, &matches21);

            for (size_t i1 = 0; i1 < matches12.size(); ++i1) {
                if (matches12[i1] != -1 && matches21[matches12[i1]] != -1 &&
                    matches21[matches12[i1]] == static_cast<int>(i1)) {
                    FeatureMatch match;
                    match.point2D_idx1 = i1;
                    match.point2D_idx2 = matches12[i1];
                    matches->push_back(match);
                }
            }
        }

        void FindBestMatches_i(SiftMatchingOptions::FeatureDistance dType, const Eigen::MatrixXi& dists, const double max_ratio,
                               FeatureMatches* matches){
            matches->clear();
            std::vector<int> matches12;
            const size_t num_matches12 = FindBestMatchesOneWay_i(dType, dists, max_ratio, &matches12);
            std::vector<int> matches21;
            const size_t num_matches21 = FindBestMatchesOneWay_i(dType, dists.transpose(), max_ratio, &matches21);

            for (size_t i1 = 0; i1 < matches12.size(); ++i1) {
                if (matches12[i1] != -1 && matches21[matches12[i1]] != -1 &&
                    matches21[matches12[i1]] == static_cast<int>(i1)) {
                    FeatureMatch match;
                    match.point2D_idx1 = i1;
                    match.point2D_idx2 = matches12[i1];
                    matches->push_back(match);
                }
            }
        }

        void FindBestMatches(const Eigen::MatrixXi cosDists, const Eigen::MatrixXd& dists, const float max_ratio,
                             const float max_distance, const bool cross_check,
                             FeatureMatches* matches) {
            matches->clear();

            std::vector<int> matches12;
            const size_t num_matches12 =
                    FindBestMatchesOneWay(cosDists, dists, max_ratio, max_distance, &matches12);

            if (cross_check) {
                std::vector<int> matches21;
                const size_t num_matches21 = FindBestMatchesOneWay(cosDists.transpose(),
                        dists.transpose(), max_ratio, max_distance, &matches21);
                matches->reserve(std::min(num_matches12, num_matches21));
                for (size_t i1 = 0; i1 < matches12.size(); ++i1) {
                    if (matches12[i1] != -1 && matches21[matches12[i1]] != -1 &&
                        matches21[matches12[i1]] == static_cast<int>(i1)) {
                        FeatureMatch match;
                        match.point2D_idx1 = i1;
                        match.point2D_idx2 = matches12[i1];
                        matches->push_back(match);
                    }
                }
            } else {
                matches->reserve(num_matches12);
                for (size_t i1 = 0; i1 < matches12.size(); ++i1) {
                    if (matches12[i1] != -1) {
                        FeatureMatch match;
                        match.point2D_idx1 = i1;
                        match.point2D_idx2 = matches12[i1];
                        matches->push_back(match);
                    }
                }
            }
        }

        void WarnIfMaxNumMatchesReachedGPU(const SiftMatchGPU& sift_match_gpu,
                                           const FeatureDescriptors& descriptors) {
            if (sift_match_gpu.GetMaxSift() < descriptors.rows()) {
                std::cout << StringPrintf(
                        "WARNING: Clamping features from %d to %d - consider "
                                "increasing the maximum number of matches.",
                        descriptors.rows(), sift_match_gpu.GetMaxSift())
                << std::endl;
            }
        }

    }  // namespace

    bool SiftMatchingOptions::Check() const {
        if (use_gpu) {
            CHECK_OPTION_GT(CSVToVector<int>(gpu_index).size(), 0);
        }
        CHECK_OPTION_GT(max_ratio, 0.0);
        CHECK_OPTION_GT(max_distance, 0.0);
        CHECK_OPTION_GT(max_error, 0.0);
        CHECK_OPTION_GT(max_num_trials, 0);
        CHECK_OPTION_GE(min_inlier_ratio, 0);
        CHECK_OPTION_LE(min_inlier_ratio, 1);
        CHECK_OPTION_GE(min_num_inliers, 0);
        return true;
    }

    FeatureMatcherCache::FeatureMatcherCache(const size_t cache_size,
                                             const Database* database)
            : cache_size_(cache_size), database_(database) {
        CHECK_NOTNULL(database_);
    }

    void FeatureMatcherCache::Setup() {
        const std::vector<Camera> cameras = database_->ReadAllCameras();
        cameras_cache_.reserve(cameras.size());
        for (const auto& camera : cameras) {
            cameras_cache_.emplace(camera.CameraId(), camera);
        }

        const std::vector<Image> images = database_->ReadAllImages();
        images_cache_.reserve(images.size());
        for (const auto& image : images) {
            images_cache_.emplace(image.ImageId(), image);
        }

        keypoints_cache_.reset(new LRUCache<image_t, FeatureKeypoints>(
                cache_size_, [this](const image_t image_id) {
                    return database_->ReadKeypoints(image_id);
                }));

        descriptors_cache_.reset(new LRUCache<image_t, FeatureDescriptors>(
                cache_size_, [this](const image_t image_id) {
                    return database_->ReadDescriptors(image_id);
                }));
    }

    const Camera& FeatureMatcherCache::GetCamera(const camera_t camera_id) const {
        return cameras_cache_.at(camera_id);
    }

    const Image& FeatureMatcherCache::GetImage(const image_t image_id) const {
        return images_cache_.at(image_id);
    }

    const FeatureKeypoints& FeatureMatcherCache::GetKeypoints(
            const image_t image_id) {
        std::unique_lock<std::mutex> lock(database_mutex_);
        return keypoints_cache_->Get(image_id);
    }

    const FeatureDescriptors& FeatureMatcherCache::GetDescriptors(
            const image_t image_id) {
        std::unique_lock<std::mutex> lock(database_mutex_);
        return descriptors_cache_->Get(image_id);
    }

    FeatureMatches FeatureMatcherCache::GetMatches(const image_t image_id1,
                                                   const image_t image_id2) {
        std::unique_lock<std::mutex> lock(database_mutex_);
        return database_->ReadMatches(image_id1, image_id2);
    }

    std::vector<image_t> FeatureMatcherCache::GetImageIds() const {
        std::vector<image_t> image_ids;
        image_ids.reserve(images_cache_.size());
        for (const auto& image : images_cache_) {
            image_ids.push_back(image.first);
        }
        return image_ids;
    }

    bool FeatureMatcherCache::ExistsMatches(const image_t image_id1,
                                            const image_t image_id2) {
        std::unique_lock<std::mutex> lock(database_mutex_);
        return database_->ExistsMatches(image_id1, image_id2);
    }
    bool FeatureMatcherCache::ExistsInlierMatches(const image_t image_id1,
                                                  const image_t image_id2) {
        std::unique_lock<std::mutex> lock(database_mutex_);
        return database_->ExistsInlierMatches(image_id1, image_id2);
    }

    void FeatureMatcherCache::WriteMatches(const image_t image_id1,
                                           const image_t image_id2,
                                           const FeatureMatches& matches) {
        std::unique_lock<std::mutex> lock(database_mutex_);
        database_->WriteMatches(image_id1, image_id2, matches);
    }
    void FeatureMatcherCache::WriteInlierMatches(
            const image_t image_id1, const image_t image_id2,
            const TwoViewGeometry& two_view_geometry) {
        std::unique_lock<std::mutex> lock(database_mutex_);
        database_->WriteInlierMatches(image_id1, image_id2, two_view_geometry);
    }

    void FeatureMatcherCache::DeleteMatches(const image_t image_id1,
                                            const image_t image_id2) {
        std::unique_lock<std::mutex> lock(database_mutex_);
        database_->DeleteMatches(image_id1, image_id2);
    }

    void FeatureMatcherCache::DeleteInlierMatches(const image_t image_id1,
                                                  const image_t image_id2) {
        std::unique_lock<std::mutex> lock(database_mutex_);
        database_->DeleteInlierMatches(image_id1, image_id2);
    }

    FeatureMatcherThread::FeatureMatcherThread(const SiftMatchingOptions& options,
                                               FeatureMatcherCache* cache)
            : options_(options), cache_(cache) {}

    void FeatureMatcherThread::SetMaxNumMatches(const int max_num_matches) {
        options_.max_num_matches = max_num_matches;
    }

    SiftCPUFeatureMatcher::SiftCPUFeatureMatcher(const SiftMatchingOptions& options,
                                                 FeatureMatcherCache* cache,
                                                 JobQueue<Input>* input_queue,
                                                 JobQueue<Output>* output_queue)
            : FeatureMatcherThread(options, cache),
              input_queue_(input_queue),
              output_queue_(output_queue) {
        CHECK(options_.Check());
    }

    void SiftCPUFeatureMatcher::Run() {
        SignalValidSetup();

        while (true) {
            if (IsStopped()) {
                break;
            }

            const auto input_job = input_queue_->Pop();
            if (input_job.IsValid()) {
                auto data = input_job.Data();

                const FeatureDescriptors descriptors1 =
                        cache_->GetDescriptors(data.image_id1);
                const FeatureDescriptors descriptors2 =
                        cache_->GetDescriptors(data.image_id2);
                MatchSiftFeaturesCPU(options_, descriptors1, descriptors2, &data.matches);

                CHECK(output_queue_->Push(data));
            }
        }
    }

    SiftGPUFeatureMatcher::SiftGPUFeatureMatcher(const SiftMatchingOptions& options,
                                                 FeatureMatcherCache* cache,
                                                 JobQueue<Input>* input_queue,
                                                 JobQueue<Output>* output_queue)
            : FeatureMatcherThread(options, cache),
              input_queue_(input_queue),
              output_queue_(output_queue) {
        CHECK(options_.Check());

        prev_uploaded_image_ids_[0] = kInvalidImageId;
        prev_uploaded_image_ids_[1] = kInvalidImageId;

#ifndef CUDA_ENABLED
        opengl_context_.reset(new OpenGLContextManager());
#endif
    }

    void SiftGPUFeatureMatcher::Run() {
#ifndef CUDA_ENABLED
        CHECK(opengl_context_);
        opengl_context_->MakeCurrent();
#endif

        SiftMatchGPU sift_match_gpu;
        if (!CreateSiftGPUMatcher(options_, &sift_match_gpu)) {
            std::cout << "ERROR: SiftGPU not fully supported" << std::endl;
            SignalInvalidSetup();
            return;
        }

        SignalValidSetup();

        while (true) {
            if (IsStopped()) {
                break;
            }

            const auto input_job = input_queue_->Pop();
            if (input_job.IsValid()) {
                auto data = input_job.Data();

                const FeatureDescriptors* descriptors1_ptr;
                GetDescriptorData(0, data.image_id1, &descriptors1_ptr);
                const FeatureDescriptors* descriptors2_ptr;
                GetDescriptorData(1, data.image_id2, &descriptors2_ptr);
                MatchSiftFeaturesGPU(options_, descriptors1_ptr, descriptors2_ptr,
                                     &sift_match_gpu, &data.matches);

                CHECK(output_queue_->Push(data));
            }
        }
    }

    void SiftGPUFeatureMatcher::GetDescriptorData(
            const int index, const image_t image_id,
            const FeatureDescriptors** descriptors_ptr) {
        CHECK_GE(index, 0);
        CHECK_LE(index, 1);
        if (prev_uploaded_image_ids_[index] == image_id) {
            *descriptors_ptr = nullptr;
        } else {
            prev_uploaded_descriptors_[index] = cache_->GetDescriptors(image_id);
            *descriptors_ptr = &prev_uploaded_descriptors_[index];
            prev_uploaded_image_ids_[index] = image_id;
        }
    }

    GuidedSiftCPUFeatureMatcher::GuidedSiftCPUFeatureMatcher(
            const SiftMatchingOptions& options, FeatureMatcherCache* cache,
            JobQueue<Input>* input_queue, JobQueue<Output>* output_queue)
            : FeatureMatcherThread(options, cache),
              input_queue_(input_queue),
              output_queue_(output_queue) {
        CHECK(options_.Check());
    }

    void GuidedSiftCPUFeatureMatcher::Run() {
        SignalValidSetup();

        while (true) {
            if (IsStopped()) {
                break;
            }

            const auto input_job = input_queue_->Pop();
            if (input_job.IsValid()) {
                auto data = input_job.Data();

                if (data.two_view_geometry.inlier_matches.size() <
                    static_cast<size_t>(options_.min_num_inliers)) {
                    CHECK(output_queue_->Push(data));
                    continue;
                }

                const FeatureKeypoints keypoints1 = cache_->GetKeypoints(data.image_id1);
                const FeatureKeypoints keypoints2 = cache_->GetKeypoints(data.image_id2);
                const FeatureDescriptors descriptors1 =
                        cache_->GetDescriptors(data.image_id1);
                const FeatureDescriptors descriptors2 =
                        cache_->GetDescriptors(data.image_id2);
                MatchGuidedSiftFeaturesCPU(options_, keypoints1, keypoints2, descriptors1,
                                           descriptors2, &data.two_view_geometry);

                CHECK(output_queue_->Push(data));
            }
        }
    }

    GuidedSiftGPUFeatureMatcher::GuidedSiftGPUFeatureMatcher(
            const SiftMatchingOptions& options, FeatureMatcherCache* cache,
            JobQueue<Input>* input_queue, JobQueue<Output>* output_queue)
            : FeatureMatcherThread(options, cache),
              input_queue_(input_queue),
              output_queue_(output_queue) {
        CHECK(options_.Check());

        prev_uploaded_image_ids_[0] = kInvalidImageId;
        prev_uploaded_image_ids_[1] = kInvalidImageId;

#ifndef CUDA_ENABLED
        opengl_context_.reset(new OpenGLContextManager());
#endif
    }

    void GuidedSiftGPUFeatureMatcher::Run() {
#ifndef CUDA_ENABLED
        CHECK(opengl_context_);
        opengl_context_->MakeCurrent();
#endif

        SiftMatchGPU sift_match_gpu;
        if (!CreateSiftGPUMatcher(options_, &sift_match_gpu)) {
            std::cout << "ERROR: SiftGPU not fully supported" << std::endl;
            SignalInvalidSetup();
            return;
        }

        SignalValidSetup();

        while (true) {
            if (IsStopped()) {
                break;
            }

            const auto input_job = input_queue_->Pop();
            if (input_job.IsValid()) {
                auto data = input_job.Data();

                if (data.two_view_geometry.inlier_matches.size() <
                    static_cast<size_t>(options_.min_num_inliers)) {
                    CHECK(output_queue_->Push(data));
                    continue;
                }

                const FeatureDescriptors* descriptors1_ptr;
                const FeatureKeypoints* keypoints1_ptr;
                GetFeatureData(0, data.image_id1, &keypoints1_ptr, &descriptors1_ptr);
                const FeatureDescriptors* descriptors2_ptr;
                const FeatureKeypoints* keypoints2_ptr;
                GetFeatureData(1, data.image_id2, &keypoints2_ptr, &descriptors2_ptr);

                MatchGuidedSiftFeaturesGPU(options_, keypoints1_ptr, keypoints2_ptr,
                                           descriptors1_ptr, descriptors2_ptr,
                                           &sift_match_gpu, &data.two_view_geometry);

                CHECK(output_queue_->Push(data));
            }
        }
    }

    void GuidedSiftGPUFeatureMatcher::GetFeatureData(
            const int index, const image_t image_id,
            const FeatureKeypoints** keypoints_ptr,
            const FeatureDescriptors** descriptors_ptr) {
        CHECK_GE(index, 0);
        CHECK_LE(index, 1);
        if (prev_uploaded_image_ids_[index] == image_id) {
            *keypoints_ptr = nullptr;
            *descriptors_ptr = nullptr;
        } else {
            prev_uploaded_keypoints_[index] = cache_->GetKeypoints(image_id);
            prev_uploaded_descriptors_[index] = cache_->GetDescriptors(image_id);
            *keypoints_ptr = &prev_uploaded_keypoints_[index];
            *descriptors_ptr = &prev_uploaded_descriptors_[index];
            prev_uploaded_image_ids_[index] = image_id;
        }
    }

    TwoViewGeometryVerifier::TwoViewGeometryVerifier(
            const SiftMatchingOptions& options, FeatureMatcherCache* cache,
            JobQueue<Input>* input_queue, JobQueue<Output>* output_queue)
            : options_(options),
              cache_(cache),
              input_queue_(input_queue),
              output_queue_(output_queue) {
        CHECK(options_.Check());

        two_view_geometry_options_.min_num_inliers =
                static_cast<size_t>(options_.min_num_inliers);
        two_view_geometry_options_.ransac_options.max_error = options_.max_error;
        two_view_geometry_options_.ransac_options.confidence = options_.confidence;
        two_view_geometry_options_.ransac_options.max_num_trials =
                static_cast<size_t>(options_.max_num_trials);
        two_view_geometry_options_.ransac_options.min_inlier_ratio =
                options_.min_inlier_ratio;
    }

    void TwoViewGeometryVerifier::Run() {
        while (true) {
            if (IsStopped()) {
                break;
            }

            const auto input_job = input_queue_->Pop();
            if (input_job.IsValid()) {
                auto data = input_job.Data();

                if (data.matches.size() < static_cast<size_t>(options_.min_num_inliers)) {
                    CHECK(output_queue_->Push(data));
                    continue;
                }

                const auto& camera1 =
                        cache_->GetCamera(cache_->GetImage(data.image_id1).CameraId());
                const auto& camera2 =
                        cache_->GetCamera(cache_->GetImage(data.image_id2).CameraId());
                const auto keypoints1 = cache_->GetKeypoints(data.image_id1);
                const auto keypoints2 = cache_->GetKeypoints(data.image_id2);
                const auto points1 = FeatureKeypointsToPointsVector(keypoints1);
                const auto points2 = FeatureKeypointsToPointsVector(keypoints2);

                if (options_.multiple_models) {
                    data.two_view_geometry.EstimateMultiple(camera1, points1, camera2,
                                                            points2, data.matches,
                                                            two_view_geometry_options_);
                } else {
                    data.two_view_geometry.Estimate(camera1, points1, camera2, points2,
                                                    data.matches,
                                                    two_view_geometry_options_);
                }

                CHECK(output_queue_->Push(data));
            }
        }
    }

    SiftFeatureMatcher::SiftFeatureMatcher(const SiftMatchingOptions& options,
                                           Database* database,
                                           FeatureMatcherCache* cache)
            : options_(options), database_(database), cache_(cache), is_setup_(false) {
        CHECK(options_.Check());

        const int num_threads = GetEffectiveNumThreads(options_.num_threads);
        CHECK_GT(num_threads, 0);

        matchers_.reserve((unsigned long) num_threads);
        for (int i = 0; i < num_threads; ++i) {
            matchers_.emplace_back(new SiftCPUFeatureMatcher(
                    options_, cache, &matcher_queue_, &verifier_queue_));
        }


        verifiers_.reserve((unsigned long) num_threads);
        if (options_.guided_matching) {
            for (int i = 0; i < num_threads; ++i) {
                verifiers_.emplace_back(new TwoViewGeometryVerifier(
                        options_, cache, &verifier_queue_, &guided_matcher_queue_));
            }

            guided_matchers_.reserve((unsigned long) num_threads);
            for (int i = 0; i < num_threads; ++i) {
                guided_matchers_.emplace_back(new GuidedSiftCPUFeatureMatcher(
                        options_, cache, &guided_matcher_queue_, &output_queue_));
            }

        } else {
            for (int i = 0; i < num_threads; ++i) {
                verifiers_.emplace_back(new TwoViewGeometryVerifier(
                        options_, cache, &verifier_queue_, &output_queue_));
            }
        }
    }

    SiftFeatureMatcher::~SiftFeatureMatcher() {
        matcher_queue_.Wait();
        verifier_queue_.Wait();
        guided_matcher_queue_.Wait();
        output_queue_.Wait();

        for (auto& matcher : matchers_) {
            matcher->Stop();
        }

        for (auto& verifier : verifiers_) {
            verifier->Stop();
        }

        for (auto& guided_matcher : guided_matchers_) {
            guided_matcher->Stop();
        }

        matcher_queue_.Stop();
        verifier_queue_.Stop();
        guided_matcher_queue_.Stop();
        output_queue_.Stop();

        for (auto& matcher : matchers_) {
            matcher->Wait();
        }

        for (auto& verifier : verifiers_) {
            verifier->Wait();
        }

        for (auto& guided_matcher : guided_matchers_) {
            guided_matcher->Wait();
        }
    }

    bool SiftFeatureMatcher::Setup() {
        const int max_num_features = CHECK_NOTNULL(database_)->MaxNumDescriptors();
        options_.max_num_matches =
                std::min(options_.max_num_matches, max_num_features);

        for (auto& matcher : matchers_) {
            matcher->SetMaxNumMatches(options_.max_num_matches);
            matcher->Start();
        }

        for (auto& verifier : verifiers_) {
            verifier->Start();
        }

        for (auto& guided_matcher : guided_matchers_) {
            guided_matcher->SetMaxNumMatches(options_.max_num_matches);
            guided_matcher->Start();
        }

        for (auto& matcher : matchers_) {
            if (!matcher->CheckValidSetup()) {
                return false;
            }
        }

        for (auto& guided_matcher : guided_matchers_) {
            if (!guided_matcher->CheckValidSetup()) {
                return false;
            }
        }

        is_setup_ = true;

        return true;
    }

    void SiftFeatureMatcher::Match(
            const std::vector<std::pair<image_t, image_t>>& image_pairs) {
        CHECK_NOTNULL(database_);
        CHECK_NOTNULL(cache_);
        CHECK(is_setup_);

        if (image_pairs.empty()) {
            return;
        }

        DatabaseTransaction database_transaction(database_);

        //////////////////////////////////////////////////////////////////////////////
        // Match the image pairs
        //////////////////////////////////////////////////////////////////////////////

        std::unordered_set<image_pair_t> image_pair_ids;
        image_pair_ids.reserve(image_pairs.size());

        size_t num_outputs = 0;
        for (const auto image_pair : image_pairs) {
            // Avoid self-matches.
            if (image_pair.first == image_pair.second) {
                continue;
            }

            // Avoid duplicate image pairs.
            const image_pair_t pair_id =
                    Database::ImagePairToPairId(image_pair.first, image_pair.second);
            if (image_pair_ids.count(pair_id) > 0) {
                continue;
            }

            image_pair_ids.insert(pair_id);

            const bool exists_matches =
                    cache_->ExistsMatches(image_pair.first, image_pair.second);
            const bool exists_inlier_matches =
                    cache_->ExistsInlierMatches(image_pair.first, image_pair.second);

            if (exists_matches && exists_inlier_matches) {
                continue;
            }

            num_outputs += 1;

            // If only one of the matches or inlier matches exist, we recompute them
            // from scratch and delete the existing results. This must be done before
            // pushing the jobs to the queue, otherwise database constraints might fail
            // when writing an existing result into the database.

            if (exists_inlier_matches) {
                cache_->DeleteInlierMatches(image_pair.first, image_pair.second);
            }

            internal::FeatureMatcherData data;
            data.image_id1 = image_pair.first;
            data.image_id2 = image_pair.second;

            if (exists_matches) {
                data.matches = cache_->GetMatches(image_pair.first, image_pair.second);
                cache_->DeleteMatches(image_pair.first, image_pair.second);
                CHECK(verifier_queue_.Push(data));
            } else {
                CHECK(matcher_queue_.Push(data));
            }
        }

        //////////////////////////////////////////////////////////////////////////////
        // Write results to database
        //////////////////////////////////////////////////////////////////////////////

        for (size_t i = 0; i < num_outputs; ++i) {
            const auto output_job = output_queue_.Pop();
            CHECK(output_job.IsValid());
            auto output = output_job.Data();

            if (output.matches.size() < static_cast<size_t>(options_.min_num_inliers)) {
                output.matches = {};
            }

            if (output.two_view_geometry.inlier_matches.size() <
                static_cast<size_t>(options_.min_num_inliers)) {
                output.two_view_geometry = TwoViewGeometry();
            }

            cache_->WriteMatches(output.image_id1, output.image_id2, output.matches);
            cache_->WriteInlierMatches(output.image_id1, output.image_id2,
                                       output.two_view_geometry);
        }

        CHECK_EQ(output_queue_.Size(), 0);
    }

    bool ExhaustiveFeatureMatcher::Options::Check() const {
        CHECK_OPTION_GT(block_size, 1);
        return true;
    }

    ExhaustiveFeatureMatcher::ExhaustiveFeatureMatcher(
            const Options& options, const SiftMatchingOptions& match_options,
            const std::string& database_path)
            : options_(options),
              match_options_(match_options),
              database_(database_path),
              cache_(5 * options_.block_size, &database_),
              matcher_(match_options, &database_, &cache_) {
        CHECK(options_.Check());
        CHECK(match_options_.Check());
    }

    void ExhaustiveFeatureMatcher::Run() {
        PrintHeading1("Exhaustive feature matching");

        if (!matcher_.Setup()) {
            return;
        }

        cache_.Setup();

        const std::vector<image_t> image_ids = cache_.GetImageIds();

        const size_t block_size = static_cast<size_t>(options_.block_size);
        const size_t num_blocks = static_cast<size_t>(
                std::ceil(static_cast<double>(image_ids.size()) / block_size));
        const size_t num_pairs_per_block = block_size * (block_size - 1) / 2;

        std::vector<std::pair<image_t, image_t>> image_pairs;
        image_pairs.reserve(num_pairs_per_block);

        for (size_t start_idx1 = 0; start_idx1 < image_ids.size();
             start_idx1 += block_size) {
            const size_t end_idx1 =
                    std::min(image_ids.size(), start_idx1 + block_size) - 1;
            for (size_t start_idx2 = 0; start_idx2 < image_ids.size();
                 start_idx2 += block_size) {
                const size_t end_idx2 =
                        std::min(image_ids.size(), start_idx2 + block_size) - 1;

                if (IsStopped()) {
                    GetTimer().PrintMinutes();
                    return;
                }

                Timer timer;
                timer.Start();

                std::cout << StringPrintf("Matching block [%d/%d, %d/%d]",
                                          start_idx1 / block_size + 1, num_blocks,
                                          start_idx2 / block_size + 1, num_blocks)
                << std::flush;

                image_pairs.clear();
                for (size_t idx1 = start_idx1; idx1 <= end_idx1; ++idx1) {
                    for (size_t idx2 = start_idx2; idx2 <= end_idx2; ++idx2) {
                        const size_t block_id1 = idx1 % block_size;
                        const size_t block_id2 = idx2 % block_size;
                        if ((idx1 > idx2 && block_id1 <= block_id2) ||
                            (idx1 < idx2 && block_id1 < block_id2)) {  // Avoid duplicate pairs
                            image_pairs.emplace_back(image_ids[idx1], image_ids[idx2]);
                        }
                    }
                }

                matcher_.Match(image_pairs);

                PrintElapsedTime(timer);
            }
        }

        GetTimer().PrintMinutes();
    }

    bool SequentialFeatureMatcher::Options::Check() const {
        CHECK_OPTION_GT(overlap, 0);
        CHECK_OPTION_GT(loop_detection_period, 0);
        CHECK_OPTION_GT(loop_detection_num_images, 0);
        CHECK_OPTION_GE(loop_detection_num_verifications, 0);
        return true;
    }

    SequentialFeatureMatcher::SequentialFeatureMatcher(
            const Options& options, const SiftMatchingOptions& match_options,
            const std::string& database_path)
            : options_(options),
              match_options_(match_options),
              database_(database_path),
              cache_(std::max(5 * options_.loop_detection_num_images,
                              5 * options_.overlap),
                     &database_),
              matcher_(match_options, &database_, &cache_) {
        CHECK(options_.Check());
        CHECK(match_options_.Check());
    }

    void SequentialFeatureMatcher::Run() {
        PrintHeading1("Sequential feature matching");

        if (!matcher_.Setup()) {
            return;
        }

        cache_.Setup();

        const std::vector<image_t> ordered_image_ids = GetOrderedImageIds();

        RunSequentialMatching(ordered_image_ids);
        if (options_.loop_detection) {
            RunLoopDetection(ordered_image_ids);
        }

        GetTimer().PrintMinutes();
    }

    std::vector<image_t> SequentialFeatureMatcher::GetOrderedImageIds() const {
        const std::vector<image_t> image_ids = cache_.GetImageIds();

        std::vector<Image> ordered_images;
        ordered_images.reserve(image_ids.size());
        for (const auto image_id : image_ids) {
            ordered_images.push_back(cache_.GetImage(image_id));
        }

        std::sort(ordered_images.begin(), ordered_images.end(),
                  [](const Image& image1, const Image& image2) {
                      return image1.Name() < image2.Name();
                  });

        std::vector<image_t> ordered_image_ids;
        ordered_image_ids.reserve(image_ids.size());
        for (const auto& image : ordered_images) {
            ordered_image_ids.push_back(image.ImageId());
        }

        return ordered_image_ids;
    }

    void SequentialFeatureMatcher::RunSequentialMatching(
            const std::vector<image_t>& image_ids) {
        std::vector<std::pair<image_t, image_t>> image_pairs;
        image_pairs.reserve(options_.overlap);

        for (size_t image_idx1 = 0; image_idx1 < image_ids.size(); ++image_idx1) {
            if (IsStopped()) {
                return;
            }

            const auto image_id1 = image_ids.at(image_idx1);

            Timer timer;
            timer.Start();

            std::cout << StringPrintf("Matching image [%d/%d]", image_idx1 + 1,
                                      image_ids.size())
            << std::flush;

            image_pairs.clear();
            for (int i = 0; i < options_.overlap; ++i) {
                const size_t image_idx2 = image_idx1 + i;
                if (image_idx2 < image_ids.size()) {
                    image_pairs.emplace_back(image_id1, image_ids.at(image_idx2));
                    if (options_.quadratic_overlap) {
                        const size_t image_idx2_quadratic = image_idx1 + (1 << i);
                        if (image_idx2_quadratic < image_ids.size()) {
                            image_pairs.emplace_back(image_id1,
                                                     image_ids.at(image_idx2_quadratic));
                        }
                    }
                } else {
                    break;
                }
            }

            matcher_.Match(image_pairs);

            PrintElapsedTime(timer);
        }
    }

    void SequentialFeatureMatcher::RunLoopDetection(
            const std::vector<image_t>& image_ids) {
        // Read the pre-trained vocabulary tree from disk.
        retrieval::VisualIndex<> visual_index;
        visual_index.Read(options_.vocab_tree_path);

        // Index all images in the visual index.
        IndexImagesInVisualIndex(match_options_.num_threads,
                                 options_.loop_detection_max_num_features, image_ids,
                                 this, &cache_, &visual_index);

        if (IsStopped()) {
            return;
        }

        // Only perform loop detection for every n-th image.
        std::vector<image_t> match_image_ids;
        for (size_t i = 0; i < image_ids.size();
             i += options_.loop_detection_period) {
            match_image_ids.push_back(image_ids[i]);
        }

        MatchNearestNeighborsInVisualIndex(
                match_options_.num_threads, options_.loop_detection_num_images,
                options_.loop_detection_num_verifications,
                options_.loop_detection_max_num_features, match_image_ids, this, &cache_,
                &visual_index, &matcher_);
    }

    bool VocabTreeFeatureMatcher::Options::Check() const {
        CHECK_OPTION_GT(num_images, 0);
        CHECK_OPTION_GE(num_verifications, 0);
        return true;
    }

    VocabTreeFeatureMatcher::VocabTreeFeatureMatcher(
            const Options& options, const SiftMatchingOptions& match_options,
            const std::string& database_path)
            : options_(options),
              match_options_(match_options),
              database_(database_path),
              cache_(5 * options_.num_images, &database_),
              matcher_(match_options, &database_, &cache_) {
        CHECK(options_.Check());
        CHECK(match_options_.Check());
    }

    void VocabTreeFeatureMatcher::Run() {
        PrintHeading1("Vocabulary tree feature matching");

        if (!matcher_.Setup()) {
            return;
        }

        cache_.Setup();

        // Read the pre-trained vocabulary tree from disk.
        retrieval::VisualIndex<> visual_index;
        visual_index.Read(options_.vocab_tree_path);

        const std::vector<image_t> all_image_ids = cache_.GetImageIds();
        std::vector<image_t> image_ids;
        if (options_.match_list_path == "") {
            image_ids = cache_.GetImageIds();
        } else {
            // Map image names to image identifiers.
            std::unordered_map<std::string, image_t> image_name_to_image_id;
            image_name_to_image_id.reserve(all_image_ids.size());
            for (const auto image_id : all_image_ids) {
                const auto& image = cache_.GetImage(image_id);
                image_name_to_image_id.emplace(image.Name(), image_id);
            }

            // Read the match list path.
            std::ifstream file(options_.match_list_path);
            CHECK(file.is_open()) << options_.match_list_path;
            std::string line;
            while (std::getline(file, line)) {
                StringTrim(&line);

                if (line.empty() || line[0] == '#') {
                    continue;
                }

                if (image_name_to_image_id.count(line) == 0) {
                    std::cerr << "ERROR: Image " << line << " does not exist." << std::endl;
                } else {
                    image_ids.push_back(image_name_to_image_id.at(line));
                }
            }
        }

        // Index all images in the visual index.
        IndexImagesInVisualIndex(match_options_.num_threads,
                                 options_.max_num_features, all_image_ids, this,
                                 &cache_, &visual_index);

        if (IsStopped()) {
            GetTimer().PrintMinutes();
            return;
        }

        // Match all images in the visual index.
        MatchNearestNeighborsInVisualIndex(
                match_options_.num_threads, options_.num_images,
                options_.num_verifications, options_.max_num_features, image_ids, this,
                &cache_, &visual_index, &matcher_);

        GetTimer().PrintMinutes();
    }

    bool SpatialFeatureMatcher::Options::Check() const {
        CHECK_OPTION_GT(max_num_neighbors, 0);
        CHECK_OPTION_GT(max_distance, 0.0);
        return true;
    }

    SpatialFeatureMatcher::SpatialFeatureMatcher(
            const Options& options, const SiftMatchingOptions& match_options,
            const std::string& database_path)
            : options_(options),
              match_options_(match_options),
              database_(database_path),
              cache_(5 * options_.max_num_neighbors, &database_),
              matcher_(match_options, &database_, &cache_) {
        CHECK(options_.Check());
        CHECK(match_options_.Check());
    }

    void SpatialFeatureMatcher::Run() {
        PrintHeading1("Spatial feature matching");

        if (!matcher_.Setup()) {
            return;
        }

        cache_.Setup();

        const std::vector<image_t> image_ids = cache_.GetImageIds();

        //////////////////////////////////////////////////////////////////////////////
        // Spatial indexing
        //////////////////////////////////////////////////////////////////////////////

        Timer timer;
        timer.Start();

        std::cout << "Indexing images..." << std::flush;

        GPSTransform gps_transform;

        size_t num_locations = 0;
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> location_matrix(
                image_ids.size(), 3);

        std::vector<size_t> location_idxs;
        location_idxs.reserve(image_ids.size());

        std::vector<Eigen::Vector3d> ells(1);

        for (size_t i = 0; i < image_ids.size(); ++i) {
            const auto image_id = image_ids[i];
            const auto& image = cache_.GetImage(image_id);

            if ((image.TvecPrior(0) == 0 && image.TvecPrior(1) == 0 &&
                 options_.ignore_z) ||
                (image.TvecPrior(0) == 0 && image.TvecPrior(1) == 0 &&
                 image.TvecPrior(2) == 0 && !options_.ignore_z)) {
                continue;
            }

            location_idxs.push_back(i);

            if (options_.is_gps) {
                ells[0](0) = image.TvecPrior(0);
                ells[0](1) = image.TvecPrior(1);
                ells[0](2) = options_.ignore_z ? 0 : image.TvecPrior(2);

                const auto xyzs = gps_transform.EllToXYZ(ells);

                location_matrix(num_locations, 0) = static_cast<float>(xyzs[0](0));
                location_matrix(num_locations, 1) = static_cast<float>(xyzs[0](1));
                location_matrix(num_locations, 2) = static_cast<float>(xyzs[0](2));
            } else {
                location_matrix(num_locations, 0) =
                        static_cast<float>(image.TvecPrior(0));
                location_matrix(num_locations, 1) =
                        static_cast<float>(image.TvecPrior(1));
                location_matrix(num_locations, 2) =
                        static_cast<float>(options_.ignore_z ? 0 : image.TvecPrior(2));
            }

            num_locations += 1;
        }

        PrintElapsedTime(timer);

        if (num_locations == 0) {
            std::cout << " => No images with location data." << std::endl;
            GetTimer().PrintMinutes();
            return;
        }

        //////////////////////////////////////////////////////////////////////////////
        // Building spatial index
        //////////////////////////////////////////////////////////////////////////////

        timer.Restart();

        std::cout << "Building search index..." << std::flush;

        flann::Matrix<float> locations(location_matrix.data(), num_locations,
                                       location_matrix.cols());

        flann::LinearIndexParams index_params;
        flann::LinearIndex<flann::L2<float>> search_index(index_params);
        search_index.buildIndex(locations);

        PrintElapsedTime(timer);

        //////////////////////////////////////////////////////////////////////////////
        // Searching spatial index
        //////////////////////////////////////////////////////////////////////////////

        timer.Restart();

        std::cout << "Searching for nearest neighbors..." << std::flush;

        const int knn = std::min<int>(options_.max_num_neighbors, num_locations);

        Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
                index_matrix(num_locations, knn);
        flann::Matrix<size_t> indices(index_matrix.data(), num_locations, knn);

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
                distance_matrix(num_locations, knn);
        flann::Matrix<float> distances(distance_matrix.data(), num_locations, knn);

        flann::SearchParams search_params(flann::FLANN_CHECKS_AUTOTUNED);
        if (match_options_.num_threads == ThreadPool::kMaxNumThreads) {
            search_params.cores = std::thread::hardware_concurrency();
        } else {
            search_params.cores = match_options_.num_threads;
        }
        if (search_params.cores <= 0) {
            search_params.cores = 1;
        }

        search_index.knnSearch(locations, indices, distances, knn, search_params);

        PrintElapsedTime(timer);

        //////////////////////////////////////////////////////////////////////////////
        // Matching
        //////////////////////////////////////////////////////////////////////////////

        const float max_distance =
                static_cast<float>(options_.max_distance * options_.max_distance);

        std::vector<std::pair<image_t, image_t>> image_pairs;
        image_pairs.reserve(knn);

        for (size_t i = 0; i < num_locations; ++i) {
            if (IsStopped()) {
                GetTimer().PrintMinutes();
                return;
            }

            timer.Restart();

            std::cout << StringPrintf("Matching image [%d/%d]", i + 1, num_locations)
            << std::flush;

            image_pairs.clear();

            for (int j = 0; j < knn; ++j) {
                // Check if query equals result.
                if (index_matrix(i, j) == i) {
                    continue;
                }

                // Since the nearest neighbors are sorted by distance, we can break.
                if (distance_matrix(i, j) > max_distance) {
                    break;
                }

                const size_t idx = location_idxs[i];
                const image_t image_id = image_ids.at(idx);
                const size_t nn_idx = location_idxs.at(index_matrix(i, j));
                const image_t nn_image_id = image_ids.at(nn_idx);
                image_pairs.emplace_back(image_id, nn_image_id);
            }

            matcher_.Match(image_pairs);

            PrintElapsedTime(timer);
        }

        GetTimer().PrintMinutes();
    }

    bool TransitiveFeatureMatcher::Options::Check() const {
        CHECK_OPTION_GT(batch_size, 0);
        CHECK_OPTION_GT(num_iterations, 0);
        return true;
    }

    TransitiveFeatureMatcher::TransitiveFeatureMatcher(
            const Options& options, const SiftMatchingOptions& match_options,
            const std::string& database_path)
            : options_(options),
              match_options_(match_options),
              database_(database_path),
              cache_(options_.batch_size, &database_),
              matcher_(match_options, &database_, &cache_) {
        CHECK(options_.Check());
        CHECK(match_options_.Check());
    }

    void TransitiveFeatureMatcher::Run() {
        PrintHeading1("Transitive feature matching");

        if (!matcher_.Setup()) {
            return;
        }

        cache_.Setup();

        const std::vector<image_t> image_ids = cache_.GetImageIds();

        std::vector<std::pair<image_t, image_t>> image_pairs;
        std::unordered_set<image_pair_t> image_pair_ids;

        for (int iteration = 0; iteration < options_.num_iterations; ++iteration) {
            if (IsStopped()) {
                GetTimer().PrintMinutes();
                return;
            }

            Timer timer;
            timer.Start();

            std::cout << StringPrintf("Iteration [%d/%d]", iteration + 1,
                                      options_.num_iterations)
            << std::endl;

            std::vector<std::pair<image_t, image_t>> existing_image_pairs;
            std::vector<int> existing_num_inliers;
            database_.ReadInlierMatchesGraph(&existing_image_pairs,
                                             &existing_num_inliers);

            CHECK_EQ(existing_image_pairs.size(), existing_num_inliers.size());

            std::unordered_map<image_t, std::vector<image_t>> adjacency;
            for (const auto& image_pair : existing_image_pairs) {
                adjacency[image_pair.first].push_back(image_pair.second);
                adjacency[image_pair.second].push_back(image_pair.first);
            }

            const size_t batch_size = static_cast<size_t>(options_.batch_size);

            size_t num_batches = 0;
            image_pairs.clear();
            image_pair_ids.clear();
            for (const auto& image : adjacency) {
                const auto image_id1 = image.first;
                for (const auto& image_id2 : image.second) {
                    if (adjacency.count(image_id2) > 0) {
                        for (const auto& image_id3 : adjacency.at(image_id2)) {
                            const auto image_pair_id =
                                    Database::ImagePairToPairId(image_id1, image_id3);
                            if (image_pair_ids.count(image_pair_id) == 0) {
                                image_pairs.emplace_back(image_id1, image_id3);
                                image_pair_ids.insert(image_pair_id);
                                if (image_pairs.size() >= batch_size) {
                                    num_batches += 1;
                                    std::cout << StringPrintf("  Batch %d", num_batches)
                                    << std::flush;
                                    matcher_.Match(image_pairs);
                                    image_pairs.clear();
                                    PrintElapsedTime(timer);
                                    timer.Restart();

                                    if (IsStopped()) {
                                        GetTimer().PrintMinutes();
                                        return;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            num_batches += 1;
            std::cout << StringPrintf("  Batch %d", num_batches) << std::flush;
            matcher_.Match(image_pairs);
            PrintElapsedTime(timer);
        }

        GetTimer().PrintMinutes();
    }

    bool ImagePairsFeatureMatcher::Options::Check() const {
        CHECK_OPTION_GT(block_size, 0);
        return true;
    }

    ImagePairsFeatureMatcher::ImagePairsFeatureMatcher(
            const Options& options, const SiftMatchingOptions& match_options,
            const std::string& database_path)
            : options_(options),
              match_options_(match_options),
              database_(database_path),
              cache_(options.block_size, &database_),
              matcher_(match_options, &database_, &cache_) {
        CHECK(options_.Check());
        CHECK(match_options_.Check());
    }

    void ImagePairsFeatureMatcher::Run() {
        PrintHeading1("Custom feature matching");

        if (!matcher_.Setup()) {
            return;
        }

        cache_.Setup();

        //////////////////////////////////////////////////////////////////////////////
        // Reading image pairs list
        //////////////////////////////////////////////////////////////////////////////

        std::unordered_map<std::string, image_t> image_name_to_image_id;
        image_name_to_image_id.reserve(cache_.GetImageIds().size());
        for (const auto image_id : cache_.GetImageIds()) {
            const auto& image = cache_.GetImage(image_id);
            image_name_to_image_id.emplace(image.Name(), image_id);
        }

        std::ifstream file(options_.match_list_path);
        CHECK(file.is_open()) << options_.match_list_path;

        std::string line;
        std::vector<std::pair<image_t, image_t>> image_pairs;
        while (std::getline(file, line)) {
            StringTrim(&line);

            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::stringstream line_stream(line);

            std::string image_name1;
            std::string image_name2;

            std::getline(line_stream, image_name1, ' ');
            StringTrim(&image_name1);
            std::getline(line_stream, image_name2, ' ');
            StringTrim(&image_name2);

            if (image_name_to_image_id.count(image_name1) == 0) {
                std::cerr << "ERROR: Image " << image_name1 << " does not exist."
                << std::endl;
                continue;
            }
            if (image_name_to_image_id.count(image_name2) == 0) {
                std::cerr << "ERROR: Image " << image_name2 << " does not exist."
                << std::endl;
                continue;
            }

            image_pairs.emplace_back(image_name_to_image_id.at(image_name1),
                                     image_name_to_image_id.at(image_name2));
        }

        //////////////////////////////////////////////////////////////////////////////
        // Feature matching
        //////////////////////////////////////////////////////////////////////////////

        const size_t num_match_blocks = image_pairs.size() / options_.block_size + 1;
        std::vector<std::pair<image_t, image_t>> block_image_pairs;
        block_image_pairs.reserve(options_.block_size);

        for (size_t i = 0; i < image_pairs.size(); i += options_.block_size) {
            if (IsStopped()) {
                GetTimer().PrintMinutes();
                return;
            }

            Timer timer;
            timer.Start();

            std::cout << StringPrintf("Matching block [%d/%d]",
                                      i / options_.block_size + 1, num_match_blocks)
            << std::flush;

            const size_t block_end = i + options_.block_size <= image_pairs.size()
                                     ? i + options_.block_size
                                     : image_pairs.size();
            std::vector<std::pair<image_t, image_t>> block_image_pairs;
            block_image_pairs.reserve(options_.block_size);
            for (size_t j = i; j < block_end; ++j) {
                block_image_pairs.push_back(image_pairs[j]);
            }

            matcher_.Match(block_image_pairs);

            PrintElapsedTime(timer);
        }

        GetTimer().PrintMinutes();
    }

    bool FeaturePairsFeatureMatcher::Options::Check() const { return true; }

    FeaturePairsFeatureMatcher::FeaturePairsFeatureMatcher(
            const Options& options, const SiftMatchingOptions& match_options,
            const std::string& database_path)
            : options_(options),
              match_options_(match_options),
              database_(database_path),
              cache_(kCacheSize, &database_) {
        CHECK(options_.Check());
        CHECK(match_options_.Check());
    }

    void FeaturePairsFeatureMatcher::Run() {
        PrintHeading1("Importing matches");

        cache_.Setup();

        std::unordered_map<std::string, const Image*> image_name_to_image;
        image_name_to_image.reserve(cache_.GetImageIds().size());
        for (const auto image_id : cache_.GetImageIds()) {
            const auto& image = cache_.GetImage(image_id);
            image_name_to_image.emplace(image.Name(), &image);
        }

        std::ifstream file(options_.match_list_path);
        CHECK(file.is_open()) << options_.match_list_path;

        std::string line;
        while (std::getline(file, line)) {
            if (IsStopped()) {
                GetTimer().PrintMinutes();
                return;
            }

            StringTrim(&line);
            if (line.empty()) {
                continue;
            }

            std::istringstream line_stream(line);

            std::string image_name1, image_name2;
            try {
                line_stream >> image_name1 >> image_name2;
            } catch (...) {
                std::cerr << "ERROR: Could not read image pair." << std::endl;
                break;
            }

            std::cout << StringPrintf("%s - %s", image_name1.c_str(),
                                      image_name2.c_str())
            << std::endl;

            if (image_name_to_image.count(image_name1) == 0) {
                std::cout << StringPrintf("SKIP: Image %s not found in database.",
                                          image_name1.c_str())
                << std::endl;
                break;
            }
            if (image_name_to_image.count(image_name2) == 0) {
                std::cout << StringPrintf("SKIP: Image %s not found in database.",
                                          image_name2.c_str())
                << std::endl;
                break;
            }

            const Image& image1 = *image_name_to_image[image_name1];
            const Image& image2 = *image_name_to_image[image_name2];

            bool skip_pair = false;
            if (database_.ExistsInlierMatches(image1.ImageId(), image2.ImageId())) {
                std::cout << "SKIP: Matches for image pair already exist in database."
                << std::endl;
                skip_pair = true;
            }

            FeatureMatches matches;
            while (std::getline(file, line)) {
                StringTrim(&line);

                if (line.empty()) {
                    break;
                }

                std::istringstream line_stream(line);

                FeatureMatch match;
                try {
                    line_stream >> match.point2D_idx1 >> match.point2D_idx2;
                } catch (...) {
                    std::cerr << "ERROR: Cannot read feature matches." << std::endl;
                    break;
                }

                matches.push_back(match);
            }

            if (skip_pair) {
                continue;
            }

            const Camera& camera1 = cache_.GetCamera(image1.CameraId());
            const Camera& camera2 = cache_.GetCamera(image2.CameraId());

            if (options_.verify_matches) {
                database_.WriteMatches(image1.ImageId(), image2.ImageId(), matches);

                const auto keypoints1 = cache_.GetKeypoints(image1.ImageId());
                const auto keypoints2 = cache_.GetKeypoints(image2.ImageId());

                TwoViewGeometry two_view_geometry;
                TwoViewGeometry::Options two_view_geometry_options;
                two_view_geometry_options.min_num_inliers =
                        static_cast<size_t>(match_options_.min_num_inliers);
                two_view_geometry_options.ransac_options.max_error =
                        match_options_.max_error;
                two_view_geometry_options.ransac_options.confidence =
                        match_options_.confidence;
                two_view_geometry_options.ransac_options.max_num_trials =
                        static_cast<size_t>(match_options_.max_num_trials);
                two_view_geometry_options.ransac_options.min_inlier_ratio =
                        match_options_.min_inlier_ratio;

                two_view_geometry.Estimate(
                        camera1, FeatureKeypointsToPointsVector(keypoints1), camera2,
                        FeatureKeypointsToPointsVector(keypoints2), matches,
                        two_view_geometry_options);

                database_.WriteInlierMatches(image1.ImageId(), image2.ImageId(),
                                             two_view_geometry);
            } else {
                TwoViewGeometry two_view_geometry;

                if (camera1.HasPriorFocalLength() && camera2.HasPriorFocalLength()) {
                    two_view_geometry.config = TwoViewGeometry::CALIBRATED;
                } else {
                    two_view_geometry.config = TwoViewGeometry::UNCALIBRATED;
                }

                two_view_geometry.inlier_matches = matches;

                database_.WriteInlierMatches(image1.ImageId(), image2.ImageId(),
                                             two_view_geometry);
            }
        }

        GetTimer().PrintMinutes();
    }

    void minitest(){
        std::cout << StringPrintf("into minitest \n");
        int size = 5;
        float *data2 = new float[128*size];
        for (unsigned int i = 0; i < size; i++){
            for(unsigned j =0; j< 128; j++){
                data2[128*i + j] = rand() % 10 + 1;
            }
        }

        float *data1 = new float[128*size];
        for (unsigned int i = 0; i < size; i++){
            for(unsigned j =0; j< 128; j++){
                data1[128*i + j] = rand() % 10 + 1;
            }
        }

        VlKDForest * kd_forest =  vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL2);
        vl_kdforest_build(kd_forest, (vl_size) size, data2);
        VlKDForestSearcher* searcher = vl_kdforest_new_searcher(kd_forest);
        VlKDForestNeighbor neighbours[2];
        for(int i=0; i < size; i++){
            float *query = new float[128];
            for(unsigned j =0; j< 128; j++){
                query[j] = rand() % 10 + 1;
            }
//            vl_size nvisited = vl_kdforestsearcher_query(searcher, &neighbours[0], 2, query);
            vl_size nvisited = vl_kdforest_query(kd_forest, &neighbours[0], 2, query);

            std::cout << StringPrintf("nvisit = %d ", nvisited)
            << StringPrintf(" neighbour 1 = %d ", neighbours[0].distance)
            << StringPrintf(" neighbour 2 = %d \n", neighbours[1].distance);
        }
        vl_kdforest_delete(kd_forest);
        std::cout << StringPrintf("out minitest \n");
    }

    size_t findBestMatchesKDTree_oneWay(std::vector<std::vector<float> > descs2,
                                      std::vector<std::vector<float> > descs1,
                                      std::vector<int>* matches){

//        float *data1 = new float[128*descs1.size()];
//        for (unsigned int i = 0; i < descs1.size(); i++){
//            std::copy(descs1[i].begin(), descs1[i].end(), data1 + 128*i);
//        }

        float *data2 = new float[128*descs2.size()];
        for (unsigned int i = 0; i < descs2.size(); i++){
            std::copy(descs2[i].begin(), descs2[i].end(), data2 + 128*i);
        }

        size_t num_match = 0;
        matches->resize(descs1.size(), -1);

        VlKDForest * kd_forest =  vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL2);
//        vl_kdforest_set_max_num_comparisons(kd_forest, 2);
//        vl_kdforest_set_thresholding_method(kd_forest, VL_KDTREE_MEDIAN);
        vl_kdforest_build(kd_forest, descs2.size(), data2);
        VlKDForestSearcher* searcher = vl_kdforest_new_searcher(kd_forest);
        VlKDForestNeighbor neighbours[2];
        for(int i=0; i < descs1.size(); i++){
            float *query = new float[128];
            std::copy(descs1[i].begin(), descs1[i].end(), query);

            vl_size nvisited = vl_kdforestsearcher_query(searcher, &neighbours[0], 2, query);
//            std::cout << StringPrintf("nvisit = %d ", nvisited)
//            << StringPrintf(" neighbour 1 = %d ", neighbours[0].distance)
//            << StringPrintf(" neighbour 2 = %d \n", neighbours[1].distance);
            if(neighbours[1].distance >0 && neighbours[1].distance* 0.8 < neighbours[0].distance){
                continue;
            }
            (*matches)[i] = (int) neighbours[0].index;
            num_match++;
        }
        vl_kdforest_delete(kd_forest);
        return num_match;
    }

    void findBestMatchesKDTree(const FeatureDescriptors& descriptors1,
                               const FeatureDescriptors& descriptors2, FeatureMatches* matches){
        matches->clear();
        minitest();
        std::vector<std::vector<float> > descriptors1_vector((unsigned long) descriptors1.rows(), std::vector<float>(128, 0.0));
        std::vector<std::vector<float> > descriptors2_vector((unsigned long) descriptors2.rows(), std::vector<float>(128, 0.0));
        for(Eigen::MatrixXi::Index i=0; i < descriptors1.rows(); i++){
            for(Eigen::MatrixXi::Index j =0; j < 128; j++){
                // chuwa tao size maf da at()
//                auto test_1 = descriptors1(i, j);
//                auto test_2 = static_cast<float>(descriptors1(i, j));
                descriptors1_vector.at(i).at(j) = static_cast<float>(descriptors1(i, j));
            }
        }

        for(Eigen::MatrixXi::Index i=0; i < descriptors2.rows(); i++){
            for(Eigen::MatrixXi::Index j =0; j < 128; j++){
                descriptors2_vector.at(i).at(j) = static_cast<float>(descriptors2(i, j));
            }
        }

        std::vector<int> matches12;
        size_t num_match12 = findBestMatchesKDTree_oneWay(descriptors2_vector, descriptors1_vector, &matches12);
        std::vector<int> matches21;
        size_t num_match21 = findBestMatchesKDTree_oneWay(descriptors1_vector, descriptors2_vector, &matches21);
        matches->reserve(std::min(num_match12, num_match21));
        for (size_t i1 = 0; i1 < matches12.size(); ++i1) {
            if (matches12[i1] != -1 && matches21[matches12[i1]] != -1 &&
                matches21[matches12[i1]] == static_cast<int>(i1)) {
                FeatureMatch match;
                match.point2D_idx1 = i1;
                match.point2D_idx2 = matches12[i1];
                matches->push_back(match);
            }
        }
    }

    void MatchSiftFeaturesCPU(const SiftMatchingOptions& match_options,
                              const FeatureDescriptors& descriptors1,
                              const FeatureDescriptors& descriptors2,
                              FeatureMatches* matches) {
        CHECK(match_options.Check());
        CHECK_NOTNULL(matches);

//        findBestMatchesKDTree(descriptors1, descriptors2, matches);
        Eigen::MatrixXd doubleDists;
        Eigen::MatrixXi intDists;

        switch(match_options.distanceType){
            case SiftMatchingOptions::FeatureDistance::EUCLIDEAN: {
                doubleDists = ComputeSiftEuclideanDistanceMatrix(nullptr, nullptr, descriptors1, descriptors2, nullptr);
                break;
            }
            case SiftMatchingOptions::FeatureDistance::CHI_SQUARE :{
                doubleDists = ComputeSiftChiSquareDistanceMatrix(nullptr, nullptr, descriptors1, descriptors2);
                break;
            }
            case SiftMatchingOptions::FeatureDistance::PEARSON_CORRELATION:{
                doubleDists = ComputeSiftPEARSON_CORRELATIONDistanceMatrix(nullptr, nullptr, descriptors1, descriptors2);
                break;
            }
            case SiftMatchingOptions::FeatureDistance::MANHATTAN:{
                intDists = ComputeSiftManhattanDistanceMatrix(nullptr, nullptr, descriptors1, descriptors2);
                break;
            }
            case SiftMatchingOptions::FeatureDistance::MINKOWSKI:{
                intDists = ComputeSiftChebyshevDistanceMatrix(nullptr, nullptr, descriptors1, descriptors2);
                break;
            }
            case SiftMatchingOptions::FeatureDistance::COSINE:{
                intDists = ComputeSiftCosinDistanceMatrix(nullptr, nullptr, descriptors1, descriptors2);
                break;
            }
        }

        switch(match_options.distanceType){
            case SiftMatchingOptions::FeatureDistance::EUCLIDEAN:
            case SiftMatchingOptions::FeatureDistance::PEARSON_CORRELATION:
            case SiftMatchingOptions::FeatureDistance::CHI_SQUARE:{
                FindBestMatches_d(match_options.distanceType, doubleDists, match_options.max_ratio, matches);
                break;
            }
            case SiftMatchingOptions::FeatureDistance::MANHATTAN:
            case SiftMatchingOptions::FeatureDistance::MINKOWSKI:
            case SiftMatchingOptions::FeatureDistance::COSINE:{
                FindBestMatches_i(match_options.distanceType, intDists, match_options.max_ratio, matches);
                break;
            }

        }


    }

    void MatchGuidedSiftFeaturesCPU(const SiftMatchingOptions& match_options,
                                    const FeatureKeypoints& keypoints1,
                                    const FeatureKeypoints& keypoints2,
                                    const FeatureDescriptors& descriptors1,
                                    const FeatureDescriptors& descriptors2,
                                    TwoViewGeometry* two_view_geometry) {
        CHECK(match_options.Check());
        CHECK_NOTNULL(two_view_geometry);

        const float max_residual = match_options.max_error * match_options.max_error;

        const Eigen::Matrix3f F = two_view_geometry->F.cast<float>();
        const Eigen::Matrix3f H = two_view_geometry->H.cast<float>();

        std::function<bool(float, float, float, float)> guided_filter;
        if (two_view_geometry->config == TwoViewGeometry::CALIBRATED ||
            two_view_geometry->config == TwoViewGeometry::UNCALIBRATED) {
            guided_filter = [&](const float x1, const float y1, const float x2,
                                const float y2) {
                const Eigen::Vector3f p1(x1, y1, 1.0f);
                const Eigen::Vector3f p2(x2, y2, 1.0f);
                const Eigen::Vector3f Fx1 = F * p1;
                const Eigen::Vector3f Ftx2 = F.transpose() * p2;
                const float x2tFx1 = p2.transpose() * Fx1;
                return x2tFx1 * x2tFx1 /
                       (Fx1(0) * Fx1(0) + Fx1(1) * Fx1(1) + Ftx2(0) * Ftx2(0) +
                        Ftx2(1) * Ftx2(1)) >
                       max_residual;
            };
        } else if (two_view_geometry->config == TwoViewGeometry::PLANAR ||
                   two_view_geometry->config == TwoViewGeometry::PANORAMIC ||
                   two_view_geometry->config ==
                   TwoViewGeometry::PLANAR_OR_PANORAMIC) {
            guided_filter = [&](const float x1, const float y1, const float x2,
                                const float y2) {
                const Eigen::Vector3f p1(x1, y1, 1.0f);
                const Eigen::Vector2f p2(x2, y2);
                return ((H * p1).hnormalized() - p2).squaredNorm() > max_residual;
            };
        } else {
            return;
        }

        CHECK(guided_filter);

        const Eigen::MatrixXd dists = ComputeSiftEuclideanDistanceMatrix(
                &keypoints1, &keypoints2, descriptors1, descriptors2, guided_filter);

        const Eigen::MatrixXi cosDists =  ComputeSiftCosinDistanceMatrix(
                &keypoints1, &keypoints2, descriptors1, descriptors2);

        FindBestMatches(cosDists, dists, match_options.max_ratio, match_options.max_distance,
                        match_options.cross_check,
                        &two_view_geometry->inlier_matches);
    }

    bool CreateSiftGPUMatcher(const SiftMatchingOptions& match_options,
                              SiftMatchGPU* sift_match_gpu) {
        CHECK(match_options.Check());
        CHECK_NOTNULL(sift_match_gpu);

        // SiftGPU uses many global static state variables and the initialization must
        // be thread-safe in order to work correctly. This is enforced here.
        static std::mutex mutex;
        std::unique_lock<std::mutex> lock(mutex);

        const std::vector<int> gpu_indices =
                CSVToVector<int>(match_options.gpu_index);
        CHECK_EQ(gpu_indices.size(), 1) << "SiftGPU can only run on one GPU";

        SiftGPU sift_gpu;
        sift_gpu.SetVerbose(0);

        *sift_match_gpu = SiftMatchGPU(match_options.max_num_matches);

#ifdef CUDA_ENABLED
        if (gpu_indices[0] >= 0) {
    sift_match_gpu->SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA_DEVICE0 +
                                gpu_indices[0]);
  } else {
    sift_match_gpu->SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA);
  }
#else   // CUDA_ENABLED
        sift_match_gpu->SetLanguage(SiftMatchGPU::SIFTMATCH_GLSL);
#endif  // CUDA_ENABLED

        if (sift_match_gpu->VerifyContextGL() == 0) {
            return false;
        }

        if (!sift_match_gpu->Allocate(match_options.max_num_matches,
                                      match_options.cross_check)) {
            std::cout << StringPrintf(
                    "ERROR: Not enough GPU memory to match %d features. "
                            "Reduce the maximum number of matches.",
                    match_options.max_num_matches)
            << std::endl;
            return false;
        }

#ifndef CUDA_ENABLED
        if (sift_match_gpu->GetMaxSift() < match_options.max_num_matches) {
            std::cout << StringPrintf(
                    "WARNING: OpenGL version of SiftGPU only supports a "
                            "maximum of %d matches - consider changing to CUDA-based "
                            "feature matching to avoid this limitation.",
                    sift_match_gpu->GetMaxSift())
            << std::endl;
        }
#endif  // CUDA_ENABLED

        return true;
    }

    void MatchSiftFeaturesGPU(const SiftMatchingOptions& match_options,
                              const FeatureDescriptors* descriptors1,
                              const FeatureDescriptors* descriptors2,
                              SiftMatchGPU* sift_match_gpu,
                              FeatureMatches* matches) {
        CHECK(match_options.Check());
        CHECK_NOTNULL(sift_match_gpu);
        CHECK_NOTNULL(matches);

        if (descriptors1 != nullptr) {
            CHECK_EQ(descriptors1->cols(), 128);
            WarnIfMaxNumMatchesReachedGPU(*sift_match_gpu, *descriptors1);
            sift_match_gpu->SetDescriptors(0, descriptors1->rows(),
                                           descriptors1->data());
        }

        if (descriptors2 != nullptr) {
            CHECK_EQ(descriptors2->cols(), 128);
            WarnIfMaxNumMatchesReachedGPU(*sift_match_gpu, *descriptors2);
            sift_match_gpu->SetDescriptors(1, descriptors2->rows(),
                                           descriptors2->data());
        }

        matches->resize(static_cast<size_t>(match_options.max_num_matches));

        const int num_matches = sift_match_gpu->GetSiftMatch(
                match_options.max_num_matches,
                reinterpret_cast<uint32_t(*)[2]>(matches->data()),
                static_cast<float>(match_options.max_distance),
                static_cast<float>(match_options.max_ratio), match_options.cross_check);

        if (num_matches < 0) {
            std::cerr << "ERROR: Feature matching failed. This is probably caused by "
                    "insufficient GPU memory. Consider reducing the maximum "
                    "number of features and/or matches."
            << std::endl;
            matches->clear();
        } else {
            CHECK_LE(num_matches, matches->size());
            matches->resize(num_matches);
        }
    }

    void MatchGuidedSiftFeaturesGPU(const SiftMatchingOptions& match_options,
                                    const FeatureKeypoints* keypoints1,
                                    const FeatureKeypoints* keypoints2,
                                    const FeatureDescriptors* descriptors1,
                                    const FeatureDescriptors* descriptors2,
                                    SiftMatchGPU* sift_match_gpu,
                                    TwoViewGeometry* two_view_geometry) {
        static_assert(sizeof(FeatureKeypoint) == 4 * sizeof(float),
                      "Invalid feature keypoint data format");

        CHECK(match_options.Check());
        CHECK_NOTNULL(sift_match_gpu);
        CHECK_NOTNULL(two_view_geometry);

        if (descriptors1 != nullptr) {
            CHECK_NOTNULL(keypoints1);
            CHECK_EQ(descriptors1->rows(), keypoints1->size());
            CHECK_EQ(descriptors1->cols(), 128);
            WarnIfMaxNumMatchesReachedGPU(*sift_match_gpu, *descriptors1);
            const size_t kIndex = 0;
            sift_match_gpu->SetDescriptors(kIndex, descriptors1->rows(),
                                           descriptors1->data());
            sift_match_gpu->SetFeautreLocation(
                    kIndex, reinterpret_cast<const float*>(keypoints1->data()), 2);
        }

        if (descriptors2 != nullptr) {
            CHECK_NOTNULL(keypoints2);
            CHECK_EQ(descriptors2->rows(), keypoints2->size());
            CHECK_EQ(descriptors2->cols(), 128);
            WarnIfMaxNumMatchesReachedGPU(*sift_match_gpu, *descriptors2);
            const size_t kIndex = 1;
            sift_match_gpu->SetDescriptors(kIndex, descriptors2->rows(),
                                           descriptors2->data());
            sift_match_gpu->SetFeautreLocation(
                    kIndex, reinterpret_cast<const float*>(keypoints2->data()), 2);
        }

        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> F;
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> H;
        float* F_ptr = nullptr;
        float* H_ptr = nullptr;
        if (two_view_geometry->config == TwoViewGeometry::CALIBRATED ||
            two_view_geometry->config == TwoViewGeometry::UNCALIBRATED) {
            F = two_view_geometry->F.cast<float>();
            F_ptr = F.data();
        } else if (two_view_geometry->config == TwoViewGeometry::PLANAR ||
                   two_view_geometry->config == TwoViewGeometry::PANORAMIC ||
                   two_view_geometry->config ==
                   TwoViewGeometry::PLANAR_OR_PANORAMIC) {
            H = two_view_geometry->H.cast<float>();
            H_ptr = H.data();
        } else {
            return;
        }

        CHECK(F_ptr != nullptr || H_ptr != nullptr);

        two_view_geometry->inlier_matches.resize(
                static_cast<size_t>(match_options.max_num_matches));

        const int num_matches = sift_match_gpu->GetGuidedSiftMatch(
                match_options.max_num_matches,
                reinterpret_cast<uint32_t(*)[2]>(
                        two_view_geometry->inlier_matches.data()),
                H_ptr, F_ptr, static_cast<float>(match_options.max_distance),
                static_cast<float>(match_options.max_ratio),
                static_cast<float>(match_options.max_error * match_options.max_error),
                static_cast<float>(match_options.max_error * match_options.max_error),
                match_options.cross_check);

        if (num_matches < 0) {
            std::cerr << "ERROR: Feature matching failed. This is probably caused by "
                    "insufficient GPU memory. Consider reducing the maximum "
                    "number of features."
            << std::endl;
            two_view_geometry->inlier_matches.clear();
        } else {
            CHECK_LE(num_matches, two_view_geometry->inlier_matches.size());
            two_view_geometry->inlier_matches.resize(num_matches);
        }
    }

}