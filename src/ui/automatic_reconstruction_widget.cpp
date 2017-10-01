//
// Created by tri on 26/09/2017.
//

#include "ui/automatic_reconstruction_widget.h"

#include "ui/main_window.h"

namespace bkmap {

    AutomaticReconstructionWidget::AutomaticReconstructionWidget(
            MainWindow* main_window)
            : OptionsWidget(main_window),
              main_window_(main_window),
              thread_control_widget_(new ThreadControlWidget(this)) {
        setWindowTitle("Automatic reconstruction");

        AddOptionDirPath(&options_.workspace_path, "Workspace folder");
        AddSpacer();
        AddOptionDirPath(&options_.image_path, "Image folder");
        AddSpacer();
        AddOptionFilePath(&options_.vocab_tree_path, "Vocabulary tree<br>(optional)");

        AddSpacer();

        QLabel* data_type_label = new QLabel(tr("Data type"), this);
        data_type_label->setFont(font());
        data_type_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        grid_layout_->addWidget(data_type_label, grid_layout_->rowCount(), 0);

        data_type_cb_ = new QComboBox(this);
        data_type_cb_->addItem("Individual images");
        data_type_cb_->addItem("Video frames");
        data_type_cb_->addItem("Internet images");
        grid_layout_->addWidget(data_type_cb_, grid_layout_->rowCount() - 1, 1);

        QLabel* quality_label = new QLabel(tr("Quality"), this);
        quality_label->setFont(font());
        quality_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        grid_layout_->addWidget(quality_label, grid_layout_->rowCount(), 0);

        quality_cb_ = new QComboBox(this);
        quality_cb_->addItem("Low");
        quality_cb_->addItem("Medium");
        quality_cb_->addItem("High");
        quality_cb_->setCurrentIndex(2);
        grid_layout_->addWidget(quality_cb_, grid_layout_->rowCount() - 1, 1);

        AddSpacer();

        AddOptionBool(&options_.single_camera, "Shared intrinsics");
        AddOptionBool(&options_.sparse, "Sparse model");
        AddOptionBool(&options_.dense, "Dense model");

        AddSpacer();

        AddOptionInt(&options_.num_threads, "num_threads", -1);
        AddOptionBool(&options_.use_gpu, "GPU");
        AddOptionText(&options_.gpu_index, "gpu_index");

        AddSpacer();

        QPushButton* run_button = new QPushButton(tr("Run"), this);
        grid_layout_->addWidget(run_button, grid_layout_->rowCount(), 1);
        connect(run_button, &QPushButton::released, this,
                &AutomaticReconstructionWidget::Run);

        render_result_ = new QAction(this);
        connect(render_result_, &QAction::triggered, this,
                &AutomaticReconstructionWidget::RenderResult, Qt::QueuedConnection);
    }

    void AutomaticReconstructionWidget::Run() {
        WriteOptions();

        if (!ExistsDir(options_.workspace_path)) {
            QMessageBox::critical(this, "", tr("Invalid workspace folder"));
            return;
        }

        if (!ExistsDir(options_.image_path)) {
            QMessageBox::critical(this, "", tr("Invalid image folder"));
            return;
        }

        switch (data_type_cb_->currentIndex()) {
            case 0:
                options_.data_type =
                        AutomaticReconstructionController::DataType::INDIVIDUAL;
                break;
            case 1:
                options_.data_type = AutomaticReconstructionController::DataType::VIDEO;
                break;
            case 2:
                options_.data_type =
                        AutomaticReconstructionController::DataType::INTERNET;
                break;
            default:
                options_.data_type =
                        AutomaticReconstructionController::DataType::INDIVIDUAL;
                break;
        }

        switch (quality_cb_->currentIndex()) {
            case 0:
                options_.quality = AutomaticReconstructionController::Quality::LOW;
                break;
            case 1:
                options_.quality = AutomaticReconstructionController::Quality::MEDIUM;
                break;
            case 2:
                options_.quality = AutomaticReconstructionController::Quality::HIGH;
                break;
            default:
                options_.quality = AutomaticReconstructionController::Quality::HIGH;
                break;
        }

        main_window_->reconstruction_manager_.Clear();
        main_window_->reconstruction_manager_widget_->Update();
        main_window_->RenderClear();
        main_window_->RenderNow();

        AutomaticReconstructionController* controller =
                new AutomaticReconstructionController(
                        options_, &main_window_->reconstruction_manager_);

        controller->AddCallback(Thread::FINISHED_CALLBACK,
                                [this]() { render_result_->trigger(); });

        thread_control_widget_->StartThread("Reconstructing...", true, controller);
    }

    void AutomaticReconstructionWidget::RenderResult() {
        if (main_window_->reconstruction_manager_.Size() > 0) {
            main_window_->reconstruction_manager_widget_->Update();
            main_window_->RenderClear();
            main_window_->RenderNow();
        }

        if (options_.sparse) {
            QMessageBox::information(
                    this, "",
                    tr("Imported the reconstructed sparse models for visualization. The "
                               "models were also exported to the <i>sparse</i> sub-folder in the "
                               "workspace."));
        }

        if (options_.dense) {
            QMessageBox::information(
                    this, "",
                    tr("To visualize the reconstructed dense point cloud, navigate to the "
                               "<i>dense</i> sub-folder in your workspace with <i>File > Import "
                               "model from...</i>. To visualize the meshed model, you must use an "
                               "external viewer such as Meshlab."));
        }
    }

}