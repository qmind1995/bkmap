//
// Created by tri on 26/09/2017.
//

#include "ui/feature_matching_widget.h"

#include "ui/options_widget.h"
#include "ui/thread_control_widget.h"

namespace bkmap {

    class FeatureMatchingTab : public OptionsWidget {
    public:
        FeatureMatchingTab(QWidget* parent, OptionManager* options);

        virtual void Run() = 0;

    protected:
        void CreateGeneralOptions();
        QComboBox* distance_type_cb_;
        OptionManager* options_;
        ThreadControlWidget* thread_control_widget_;
    };

    class ExhaustiveMatchingTab : public FeatureMatchingTab {
    public:
        ExhaustiveMatchingTab(QWidget* parent, OptionManager* options);
        void Run() override;
    };

    class SequentialMatchingTab : public FeatureMatchingTab {
    public:
        SequentialMatchingTab(QWidget* parent, OptionManager* options);
        void Run() override;
    };

    class VocabTreeMatchingTab : public FeatureMatchingTab {
    public:
        VocabTreeMatchingTab(QWidget* parent, OptionManager* options);
        void Run() override;
    };

    class SpatialMatchingTab : public FeatureMatchingTab {
    public:
        SpatialMatchingTab(QWidget* parent, OptionManager* options);
        void Run() override;
    };

    class TransitiveMatchingTab : public FeatureMatchingTab {
    public:
        TransitiveMatchingTab(QWidget* parent, OptionManager* options);
        void Run() override;
    };

    class CustomMatchingTab : public FeatureMatchingTab {
    public:
        CustomMatchingTab(QWidget* parent, OptionManager* options);
        void Run() override;

    private:
        std::string match_list_path_;
        QComboBox* match_type_cb_;
    };

    FeatureMatchingTab::FeatureMatchingTab(QWidget* parent, OptionManager* options)
            : OptionsWidget(parent),
              options_(options),
              thread_control_widget_(new ThreadControlWidget(this)) {}

    void FeatureMatchingTab::CreateGeneralOptions() {
//        AddSpacer();
//        AddSpacer();
//        AddSection("General Options");
        AddSpacer();
        unsigned int n = std::thread::hardware_concurrency();
        options_->sift_matching->num_threads = n-2;
        AddOptionBool(&options_->sift_matching->use_gpu, "use_gpu", true);
        AddOptionText(&options_->sift_matching->gpu_index, "gpu_index", true);

        distance_type_cb_ = AddDistanceOption();

        AddSpacer();

        QPushButton* run_button = new QPushButton(tr("Run"), this);
        grid_layout_->addWidget(run_button, grid_layout_->rowCount(), 1);
        connect(run_button, &QPushButton::released, this, &FeatureMatchingTab::Run);
    }

    ExhaustiveMatchingTab::ExhaustiveMatchingTab(QWidget* parent,
                                                 OptionManager* options)
            : FeatureMatchingTab(parent, options) {
        AddOptionInt(&options_->exhaustive_matching->block_size, "block_size", 2, 10, true);

        CreateGeneralOptions();
    }

    void ExhaustiveMatchingTab::Run() {
        WriteOptions();

        Thread* matcher = new ExhaustiveFeatureMatcher(*options_->exhaustive_matching,
                                                       *options_->sift_matching,
                                                       *options_->database_path);

        int index = distance_type_cb_->currentIndex();
        switch (index){
            case 0:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::EUCLIDEAN;
                break;
            }
            case 1:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::MANHATTAN;
                break;
            }
            case 2:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::MINKOWSKI;
                break;
            }
            case 3:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::COSINE;
                break;
            }
            case 4:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::CHI_SQUARE;
                break;
            }
            case 5:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::PEARSON_CORRELATION;
                break;
            }
            default:break;
        }

        thread_control_widget_->StartThread("Matching...", true, matcher);
    }

    SequentialMatchingTab::SequentialMatchingTab(QWidget* parent,
                                                 OptionManager* options)
            : FeatureMatchingTab(parent, options) {
        AddOptionInt(&options_->sequential_matching->overlap, "overlap", 1, 10, true);
        AddOptionBool(&options_->sequential_matching->quadratic_overlap,
                      "quadratic_overlap");
        AddOptionBool(&options_->sequential_matching->loop_detection,
                      "loop_detection");
        AddOptionInt(&options_->sequential_matching->loop_detection_period,
                     "loop_detection_period");
        AddOptionInt(&options_->sequential_matching->loop_detection_num_images,
                     "loop_detection_num_images");
        AddOptionInt(&options_->sequential_matching->loop_detection_num_verifications,
                     "loop_detection_num_verifications");
        AddOptionInt(&options_->sequential_matching->loop_detection_max_num_features,
                     "loop_detection_max_num_features", -1);
        AddOptionFilePath(&options_->sequential_matching->vocab_tree_path,
                          "vocab_tree_path");

        CreateGeneralOptions();
    }

    void SequentialMatchingTab::Run() {
        WriteOptions();

        if (options_->sequential_matching->loop_detection &&
            !ExistsFile(options_->sequential_matching->vocab_tree_path)) {
            QMessageBox::critical(this, "", tr("Invalid vocabulary tree path."));
            return;
        }

        int index = distance_type_cb_->currentIndex();
        switch (index){
            case 0:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::EUCLIDEAN;
                break;
            }
            case 1:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::MANHATTAN;
                break;
            }
            case 2:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::MINKOWSKI;
                break;
            }
            case 3:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::COSINE;
                break;
            }
            case 4:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::CHI_SQUARE;
                break;
            }
            case 5:{
                options_->sift_matching->distanceType = SiftMatchingOptions::FeatureDistance::PEARSON_CORRELATION;
                break;
            }
            default:break;
        }

        Thread* matcher = new SequentialFeatureMatcher(*options_->sequential_matching,
                                                       *options_->sift_matching,
                                                       *options_->database_path);
        thread_control_widget_->StartThread("Matching...", true, matcher);
    }

    VocabTreeMatchingTab::VocabTreeMatchingTab(QWidget* parent,
                                               OptionManager* options)
            : FeatureMatchingTab(parent, options) {
        AddOptionInt(&options_->vocab_tree_matching->num_images, "num_images");
        AddOptionInt(&options_->vocab_tree_matching->num_verifications,
                     "num_verifications");
        AddOptionInt(&options_->vocab_tree_matching->max_num_features,
                     "max_num_features", -1);
        AddOptionFilePath(&options_->vocab_tree_matching->vocab_tree_path,
                          "vocab_tree_path");

        CreateGeneralOptions();
    }

    void VocabTreeMatchingTab::Run() {
        WriteOptions();

        if (!ExistsFile(options_->vocab_tree_matching->vocab_tree_path)) {
            QMessageBox::critical(this, "", tr("Invalid vocabulary tree path."));
            return;
        }

        Thread* matcher = new VocabTreeFeatureMatcher(*options_->vocab_tree_matching,
                                                      *options_->sift_matching,
                                                      *options_->database_path);
        thread_control_widget_->StartThread("Matching...", true, matcher);
    }

    SpatialMatchingTab::SpatialMatchingTab(QWidget* parent, OptionManager* options)
            : FeatureMatchingTab(parent, options) {
        AddOptionBool(&options_->spatial_matching->is_gps, "is_gps");
        AddOptionBool(&options_->spatial_matching->ignore_z, "ignore_z");
        AddOptionInt(&options_->spatial_matching->max_num_neighbors,
                     "max_num_neighbors");
        AddOptionDouble(&options_->spatial_matching->max_distance, "max_distance");

        CreateGeneralOptions();
    }

    void SpatialMatchingTab::Run() {
        WriteOptions();

        Thread* matcher = new SpatialFeatureMatcher(*options_->spatial_matching,
                                                    *options_->sift_matching,
                                                    *options_->database_path);
        thread_control_widget_->StartThread("Matching...", true, matcher);
    }

    TransitiveMatchingTab::TransitiveMatchingTab(QWidget* parent,
                                                 OptionManager* options)
            : FeatureMatchingTab(parent, options) {
        AddOptionInt(&options->transitive_matching->batch_size, "batch_size");
        AddOptionInt(&options->transitive_matching->num_iterations, "num_iterations");

        CreateGeneralOptions();
    }

    void TransitiveMatchingTab::Run() {
        WriteOptions();

        Thread* matcher = new TransitiveFeatureMatcher(*options_->transitive_matching,
                                                       *options_->sift_matching,
                                                       *options_->database_path);
        thread_control_widget_->StartThread("Matching...", true, matcher);
    }

    CustomMatchingTab::CustomMatchingTab(QWidget* parent, OptionManager* options)
            : FeatureMatchingTab(parent, options) {
        match_type_cb_ = new QComboBox(this);
        match_type_cb_->addItem(QString("Image pairs"));
        match_type_cb_->addItem(QString("Raw feature matches"));
        match_type_cb_->addItem(QString("Inlier feature matches"));
        grid_layout_->addWidget(match_type_cb_, grid_layout_->rowCount(), 1);

        AddOptionFilePath(&match_list_path_, "match_list_path");

        CreateGeneralOptions();
    }

    void CustomMatchingTab::Run() {
        WriteOptions();

        if (!ExistsFile(match_list_path_)) {
            QMessageBox::critical(this, "", tr("Path does not exist!"));
            return;
        }

        Thread* matcher = nullptr;
        if (match_type_cb_->currentIndex() == 0) {
            ImagePairsFeatureMatcher::Options matcher_options;
            matcher_options.match_list_path = match_list_path_;
            matcher = new ImagePairsFeatureMatcher(
                    matcher_options, *options_->sift_matching, *options_->database_path);
        } else {
            FeaturePairsFeatureMatcher::Options matcher_options;
            matcher_options.match_list_path = match_list_path_;
            if (match_type_cb_->currentIndex() == 1) {
                matcher_options.verify_matches = true;
            } else if (match_type_cb_->currentIndex() == 2) {
                matcher_options.verify_matches = false;
            }

            matcher = new FeaturePairsFeatureMatcher(
                    matcher_options, *options_->sift_matching, *options_->database_path);
        }

        thread_control_widget_->StartThread("Matching...", true, matcher);
    }

    FeatureMatchingWidget::FeatureMatchingWidget(QWidget* parent,
                                                 OptionManager* options)
            : parent_(parent) {
        // Do not change flag, to make sure feature database is not accessed from
        // multiple threads
        setWindowFlags(Qt::Window);
        setWindowTitle("Feature matching");

        QGridLayout* grid = new QGridLayout(this);

        tab_widget_ = new QTabWidget(this);
        tab_widget_->addTab(new ExhaustiveMatchingTab(this, options),
                            tr("Exhaustive"));
        tab_widget_->addTab(new SequentialMatchingTab(this, options),
                            tr("Sequential"));

        grid->addWidget(tab_widget_, 0, 0);
    }

    void FeatureMatchingWidget::showEvent(QShowEvent* event) {
        parent_->setDisabled(true);
    }

    void FeatureMatchingWidget::hideEvent(QHideEvent* event) {
        parent_->setEnabled(true);
    }

}