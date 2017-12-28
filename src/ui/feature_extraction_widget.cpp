//
// Created by tri on 26/09/2017.
//

#include "ui/feature_extraction_widget.h"

#include "ui/options_widget.h"
#include "ui/qt_utils.h"
#include "ui/thread_control_widget.h"

namespace bkmap {

    class ExtractionWidget : public OptionsWidget {
    public:
        ExtractionWidget(QWidget* parent, OptionManager* options);

        virtual void Run() = 0;

    protected:
        OptionManager* options_;
        ThreadControlWidget* thread_control_widget_;
    };

    class SIFTExtractionWidget : public ExtractionWidget {
    public:
        SIFTExtractionWidget(QWidget* parent, OptionManager* options);

        void Run() override;

    private:
        QRadioButton* sift_gpu_;
        QRadioButton* sift_cpu_;
    };

    class ImportFeaturesWidget : public ExtractionWidget {
    public:
        ImportFeaturesWidget(QWidget* parent, OptionManager* options);

        void Run() override;

    private:
        std::string import_path_;
    };

    ExtractionWidget::ExtractionWidget(QWidget* parent, OptionManager* options)
            : OptionsWidget(parent),
              options_(options),
              thread_control_widget_(new ThreadControlWidget(this)) {}

    SIFTExtractionWidget::SIFTExtractionWidget(QWidget* parent,
                                               OptionManager* options)
            : ExtractionWidget(parent, options) {
        sift_gpu_ = new QRadioButton(tr("GPU"), this);

        grid_layout_->addWidget(sift_gpu_);
        grid_layout_->addWidget(sift_gpu_, grid_layout_->rowCount(), 1);
        sift_gpu_->hide();

        sift_cpu_ = new QRadioButton(tr("CPU"), this);
        sift_cpu_->setChecked(true);
        grid_layout_->addWidget(sift_cpu_, grid_layout_->rowCount(), 1);
        sift_cpu_->hide();
        AddSpacer();
        unsigned int n = std::thread::hardware_concurrency();
        options->sift_extraction->num_threads = n -2;

        AddOptionBool(&options->sift_extraction->use_gpu, "use_gpu", true);
        AddOptionText(&options->sift_extraction->gpu_index, "gpu_index", true);
    }

    void SIFTExtractionWidget::Run() {
        WriteOptions();

        ImageReader::Options reader_options = *options_->image_reader;
        reader_options.database_path = *options_->database_path;
        reader_options.image_path = *options_->image_path;

        Thread* extractor =
                new SiftFeatureExtractor(reader_options, *options_->sift_extraction);
        thread_control_widget_->StartThread("Extracting...", true, extractor);
    }

    ImportFeaturesWidget::ImportFeaturesWidget(QWidget* parent,
                                               OptionManager* options)
            : ExtractionWidget(parent, options) {
        AddOptionDirPath(&import_path_, "import_path");
    }

    void ImportFeaturesWidget::Run() {
        WriteOptions();

        if (!ExistsDir(import_path_)) {
            QMessageBox::critical(this, "", tr("Path is not a directory"));
            return;
        }

        ImageReader::Options reader_options = *options_->image_reader;
        reader_options.database_path = *options_->database_path;
        reader_options.image_path = *options_->image_path;

        Thread* importer = new FeatureImporter(reader_options, import_path_);
        thread_control_widget_->StartThread("Importing...", true, importer);
    }

    FeatureExtractionWidget::FeatureExtractionWidget(QWidget* parent,
                                                     OptionManager* options)
            : parent_(parent), options_(options) {
        // Do not change flag, to make sure feature database is not accessed from
        // multiple threads
        setWindowFlags(Qt::Window);
        setWindowTitle("Feature extraction");

        QGridLayout* grid = new QGridLayout(this);

        camera_params_text_ = new QLineEdit(this);
        camera_params_text_->setEnabled(false);
        camera_params_text_->hide();
        single_camera_cb_ = new QCheckBox("Shared for all images", this);
        single_camera_cb_->setChecked(false);
        grid->addWidget(camera_params_text_);
        grid->addWidget(single_camera_cb_);
//        grid->addWidget(CreateCameraModelBox(), 0, 0);

        tab_widget_ = new QTabWidget(this);
        tab_widget_->addTab(new SIFTExtractionWidget(this, options), tr("Extract"));
        grid->addWidget(tab_widget_);


        //extract btn
        QPushButton* extract_button = new QPushButton(tr("Extract"), this);
        connect(extract_button, &QPushButton::released, this,
                &FeatureExtractionWidget::Extract);
        grid->addWidget(extract_button, grid->rowCount(), 0);
    }


    void FeatureExtractionWidget::showEvent(QShowEvent* event) {
        parent_->setDisabled(true);
        ReadOptions();
    }

    void FeatureExtractionWidget::hideEvent(QHideEvent* event) {
        parent_->setEnabled(true);
        WriteOptions();
    }

    void FeatureExtractionWidget::ReadOptions() {
        single_camera_cb_->setChecked(options_->image_reader->single_camera);
        camera_params_text_->setText(
                QString::fromStdString(options_->image_reader->camera_params));
    }

    void FeatureExtractionWidget::WriteOptions() {
        options_->image_reader->single_camera = single_camera_cb_->isChecked();
        options_->image_reader->camera_params =
                camera_params_text_->text().toUtf8().constData();
    }

    void FeatureExtractionWidget::SelectCameraModel(const int idx) {
        const int code = camera_model_ids_[idx];
        camera_params_info_->setText(QString::fromStdString(StringPrintf(
                "<small>Parameters: %s</small>", CameraModelParamsInfo(code).c_str())));
    }

    void FeatureExtractionWidget::Extract() {

        WriteOptions();

        static_cast<ExtractionWidget*>(tab_widget_->currentWidget())->Run();
    }

}