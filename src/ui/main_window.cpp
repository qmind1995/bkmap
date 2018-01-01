//
// Created by tri on 26/09/2017.
//

#include "ui/main_window.h"

#include "util/version.h"

namespace bkmap {

    MainWindow::MainWindow(const OptionManager& options)
            : options_(options),
              thread_control_widget_(new ThreadControlWidget(this)),
              window_closed_(false) {
        resize(1024, 600);
        UpdateWindowTitle();

        CreateWidgets();
        CreateActions();
        CreateToolbar();
        CreateStatusbar();
        CreateControllers();
        ShowLog();
        options_.AddAllOptions();
    }

    const ReconstructionManager& MainWindow::GetReconstructionManager() const {
        return reconstruction_manager_;
    }

    void MainWindow::showEvent(QShowEvent* event) {
        after_show_event_->trigger();
        event->accept();
    }

    void MainWindow::afterShowEvent() { opengl_window_->PaintGL(); }

    void MainWindow::closeEvent(QCloseEvent* event) {
        if (window_closed_) {
            event->accept();
            return;
        }

        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "", tr("Do you really want to quit?"),
                                      QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No) {
            event->ignore();
        } else {
            if (mapper_controller_) {
                mapper_controller_->Stop();
                mapper_controller_->Wait();
            }

            log_widget_->close();
            event->accept();

            window_closed_ = true;
        }
    }

    void MainWindow::CreateWidgets() {
        opengl_window_ = new OpenGLWindow(this, &options_);

        #ifdef _MSC_VER
                setCentralWidget(QWidget::createWindowContainer(opengl_window_, this,
                                                          Qt::MSWindowsOwnDC));
        #else
                setCentralWidget(QWidget::createWindowContainer(opengl_window_, this));
        #endif

        project_widget_ = new ProjectWidget(this, &options_);
        project_widget_->SetDatabasePath(*options_.database_path);
        project_widget_->SetImagePath(*options_.image_path);

        feature_extraction_widget_ = new FeatureExtractionWidget(this, &options_);
        feature_matching_widget_ = new FeatureMatchingWidget(this, &options_);
        log_widget_ = new LogWidget(this);
        reconstruction_manager_widget_ =
                new ReconstructionManagerWidget(this, &reconstruction_manager_);
        reconstruction_manager_widget_->hide();

        dock_log_widget_ = new QDockWidget("", this);
        dock_log_widget_->setWidget(log_widget_);
        addDockWidget(Qt::RightDockWidgetArea, dock_log_widget_);
    }

    void MainWindow::CreateActions() {
        after_show_event_ = new QAction(tr("After show event"), this);
        connect(after_show_event_, &QAction::triggered, this, &MainWindow::afterShowEvent, Qt::QueuedConnection);

        //////////////////////////////////////////////////////////////////////////////
        // File actions
        //////////////////////////////////////////////////////////////////////////////

        action_project_new_ = new QAction(QIcon(":/media/project-new.png"), tr("New project"), this);
        action_project_new_->setShortcuts(QKeySequence::New);
        connect(action_project_new_, &QAction::triggered, this, &MainWindow::ProjectNew);

        action_quit_ = new QAction(tr("Quit"), this);
        connect(action_quit_, &QAction::triggered, this, &MainWindow::close);

        //////////////////////////////////////////////////////////////////////////////
        // Processing action
        //////////////////////////////////////////////////////////////////////////////

        action_feature_extraction_ = new QAction(
                QIcon(":/media/feature-extraction.png"), tr("Feature extraction"), this);
        connect(action_feature_extraction_, &QAction::triggered, this,
                &MainWindow::FeatureExtraction);
        blocking_actions_.push_back(action_feature_extraction_);

        action_render_camera = new QAction(QIcon(":/media/photo-camera.png"), tr("Render Camera"), this);
        connect(action_render_camera, &QAction::triggered, this,
                &MainWindow::RenderCamera);
        blocking_actions_.push_back(action_render_camera);

        action_analyze_object = new QAction(QIcon(":/media/grab-image.png"), tr("show keypoints"), this);
        connect(action_analyze_object, &QAction::triggered, this,
                &MainWindow::ShowAnalyze);
        blocking_actions_.push_back(action_analyze_object);

        action_export_ply = new QAction(QIcon(":/media/export-all.png"), tr("Export .ply"), this);
        connect(action_export_ply, &QAction::triggered, this,
                &MainWindow::ExportPlyFile);
        blocking_actions_.push_back(action_export_ply);


        action_feature_matching_ = new QAction(QIcon(":/media/feature-matching.png"), tr("Feature matching"), this);
        connect(action_feature_matching_, &QAction::triggered, this,
                &MainWindow::FeatureMatching);
        blocking_actions_.push_back(action_feature_matching_);

        //////////////////////////////////////////////////////////////////////////////
        // Reconstruction actions
        //////////////////////////////////////////////////////////////////////////////

        action_reconstruction_start_ = new QAction(QIcon(":/media/reconstruction-start.png"), tr("Start reconstruction"), this);
        connect(action_reconstruction_start_, &QAction::triggered, this, &MainWindow::ReconstructionStart);
        blocking_actions_.push_back(action_reconstruction_start_);

        action_reconstruction_pause_ = new QAction(QIcon(":/media/reconstruction-pause.png"), tr("Pause reconstruction"), this);
        connect(action_reconstruction_pause_, &QAction::triggered, this, &MainWindow::ReconstructionPause);
        action_reconstruction_pause_->setEnabled(false);
        blocking_actions_.push_back(action_reconstruction_pause_);

        action_reconstruction_reset_ = new QAction(QIcon(":/media/reconstruction-reset.png"), tr("Reset reconstruction"), this);
        connect(action_reconstruction_reset_, &QAction::triggered, this, &MainWindow::ReconstructionOverwrite);

        //////////////////////////////////////////////////////////////////////////////
        // Render actions
        //////////////////////////////////////////////////////////////////////////////

        action_render_reset_view_ = new QAction(
                QIcon(":/media/render-reset-view.png"), tr("Reset view"), this);
        connect(action_render_reset_view_, &QAction::triggered, opengl_window_,
                &OpenGLWindow::ResetView);

        connect(
                reconstruction_manager_widget_,
                static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                this, &MainWindow::SelectReconstructionIdx);

        //////////////////////////////////////////////////////////////////////////////
        // Extras actions
        //////////////////////////////////////////////////////////////////////////////

        action_log_show_ =
                new QAction(QIcon(":/media/log.png"), tr("Show log"), this);
        connect(action_log_show_, &QAction::triggered, this, &MainWindow::ShowLog);

        //////////////////////////////////////////////////////////////////////////////
        // Misc actions
        //////////////////////////////////////////////////////////////////////////////

        action_render_ = new QAction(tr("Render"), this);
        connect(action_render_, &QAction::triggered, this, &MainWindow::Render,
                Qt::BlockingQueuedConnection);

        action_render_now_ = new QAction(tr("Render now"), this);
        connect(action_render_now_, &QAction::triggered, this, &MainWindow::RenderNow,
                Qt::BlockingQueuedConnection);

        action_reconstruction_finish_ =
                new QAction(tr("Finish reconstruction"), this);
        connect(action_reconstruction_finish_, &QAction::triggered, this,
                &MainWindow::ReconstructionFinish, Qt::BlockingQueuedConnection);

    }

    void MainWindow::CreateToolbar() {
        file_toolbar_ = addToolBar(tr("File"));
        file_toolbar_->addAction(action_project_new_);
        file_toolbar_->setIconSize(QSize(16, 16));

        preprocessing_toolbar_ = addToolBar(tr("Processing"));
        preprocessing_toolbar_->addAction(action_feature_extraction_);
        preprocessing_toolbar_->addAction(action_analyze_object);
        preprocessing_toolbar_->addAction(action_feature_matching_);
        preprocessing_toolbar_->setIconSize(QSize(16, 16));

        reconstruction_toolbar_ = addToolBar(tr("Reconstruction"));
        reconstruction_toolbar_->addAction(action_reconstruction_start_);
        reconstruction_toolbar_->addAction(action_reconstruction_pause_);
        reconstruction_toolbar_->addAction(action_render_camera);
        reconstruction_toolbar_->addAction(action_export_ply);
        reconstruction_toolbar_->setIconSize(QSize(16, 16));
    }

    void MainWindow::CreateStatusbar() {
        QFont font;
        font.setPointSize(11);

        statusbar_timer_label_ = new QLabel("Time 00:00:00:00", this);
        statusbar_timer_label_->setFont(font);
        statusbar_timer_label_->setAlignment(Qt::AlignCenter);
        statusBar()->addWidget(statusbar_timer_label_, 1);
        statusbar_timer_ = new QTimer(this);
        connect(statusbar_timer_, &QTimer::timeout, this, &MainWindow::UpdateTimer);
        statusbar_timer_->start(1000);

        opengl_window_->statusbar_status_label =
                new QLabel("0 Images - 0 Points", this);
        opengl_window_->statusbar_status_label->setFont(font);
        opengl_window_->statusbar_status_label->setAlignment(Qt::AlignCenter);
        statusBar()->addWidget(opengl_window_->statusbar_status_label, 1);
    }

    void MainWindow::CreateControllers() {
        if (mapper_controller_) {
            mapper_controller_->Stop();
            mapper_controller_->Wait();
        }

        mapper_controller_.reset(new IncrementalMapperController(
                options_.mapper.get(), *options_.image_path, *options_.database_path,
                &reconstruction_manager_));
        mapper_controller_->AddCallback(
                IncrementalMapperController::INITIAL_IMAGE_PAIR_REG_CALLBACK, [this]() {
                    if (!mapper_controller_->IsStopped()) {
                        action_render_now_->trigger();
                    }
                });
        mapper_controller_->AddCallback(
                IncrementalMapperController::NEXT_IMAGE_REG_CALLBACK, [this]() {
                    if (!mapper_controller_->IsStopped()) {
                        action_render_->trigger();
                    }
                });
        mapper_controller_->AddCallback(
                IncrementalMapperController::LAST_IMAGE_REG_CALLBACK, [this]() {
                    if (!mapper_controller_->IsStopped()) {
                        action_render_now_->trigger();
                    }
//                    WriteAnalysisFile();
                });
        mapper_controller_->AddCallback(
                IncrementalMapperController::FINISHED_CALLBACK, [this]() {
                    if (!mapper_controller_->IsStopped()) {
                        action_render_now_->trigger();
                        action_reconstruction_finish_->trigger();
                    }
                    if (reconstruction_manager_.Size() == 0) {
                        action_reconstruction_reset_->trigger();
                    }
                });
    }

    void MainWindow::ProjectNew() {
        if (ReconstructionOverwrite()) {
            project_widget_->Reset();
            project_widget_->show();
            project_widget_->raise();
        }
    }

    void MainWindow::FeatureExtraction() {
        if (options_.Check()) {
            feature_extraction_widget_->show();
            feature_extraction_widget_->raise();
        } else {
            ShowInvalidProjectError();
        }
    }

    void MainWindow::ShowAnalyze(){
        // call show object analyze in opengl window
        opengl_window_->ShowObjectAnalyze(&options_);
    }

    void MainWindow::ExportPlyFile(){
        const size_t reconstruction_idx = SelectedReconstructionIdx();
        Reconstruction* currReconstruction_ = &reconstruction_manager_.Get(reconstruction_idx);

        const std::string pointcloud_path =
                QFileDialog::getSaveFileName(this, tr("Select path to point cloud file"), "",
                                             tr("pointCloud (*.ply)"))
                        .toUtf8()
                        .constData();

        if (pointcloud_path == "") {
            return;
        }

        currReconstruction_->writePointCloudFile(pointcloud_path);
    }

    void MainWindow::RenderCamera(){
        opengl_window_->onCamera();
        if(HasSelectedReconstruction()){
            const size_t reconstruction_idx = SelectedReconstructionIdx();
            opengl_window_->reconstruction = &reconstruction_manager_.Get(reconstruction_idx);
            opengl_window_->Update();
        }
    }

    void MainWindow::FeatureMatching() {
        if (options_.Check()) {
            feature_matching_widget_->show();
            feature_matching_widget_->raise();
        } else {
            ShowInvalidProjectError();
        }
    }

    void MainWindow::ReconstructionStart() {
        if (!mapper_controller_->IsStarted() && !options_.Check()) {
            ShowInvalidProjectError();
            return;
        }

        if (mapper_controller_->IsFinished() && HasSelectedReconstruction()) {
            QMessageBox::critical(this, "",
                                  tr("Reset reconstruction before starting."));
            return;
        }

        if (mapper_controller_->IsStarted()) {
            // Resume existing reconstruction.
            timer_.Resume();
            mapper_controller_->Resume();
        } else {
            // Start new reconstruction.
            CreateControllers();
            timer_.Restart();
            mapper_controller_->Start();
            action_reconstruction_start_->setText(tr("Resume reconstruction"));
        }

        DisableBlockingActions();
        action_reconstruction_pause_->setEnabled(true);
    }

    void MainWindow::ReconstructionPause() {
        timer_.Pause();
        mapper_controller_->Pause();
        EnableBlockingActions();
        action_reconstruction_pause_->setEnabled(false);
    }

    void MainWindow::ReconstructionFinish() {
        timer_.Pause();
        mapper_controller_->Stop();
        EnableBlockingActions();
        action_reconstruction_start_->setEnabled(false);
        action_reconstruction_pause_->setEnabled(false);
    }

    void MainWindow::ReconstructionReset() {
        CreateControllers();

        reconstruction_manager_.Clear();
        reconstruction_manager_widget_->Update();

        timer_.Reset();
        UpdateTimer();

        EnableBlockingActions();
        action_reconstruction_start_->setText(tr("Start reconstruction"));
        action_reconstruction_pause_->setEnabled(false);

        RenderClear();
    }

    bool MainWindow::ReconstructionOverwrite() {
        if (reconstruction_manager_.Size() == 0) {
            ReconstructionReset();
            return true;
        }

        QMessageBox::StandardButton reply = QMessageBox::question(
                this, "",
                tr("Do you really want to overwrite the existing reconstruction?"),
                QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No) {
            return false;
        } else {
            ReconstructionReset();
            return true;
        }
    }

    void MainWindow::Render() {
        if (reconstruction_manager_.Size() == 0) {
            return;
        }

        RenderNow();
    }

    void MainWindow::RenderNow() {
        reconstruction_manager_widget_->Update();
        RenderSelectedReconstruction();
    }

    void MainWindow::RenderSelectedReconstruction() {
        if (reconstruction_manager_.Size() == 0) {
            RenderClear();
            return;
        }

        const size_t reconstruction_idx = SelectedReconstructionIdx();
        opengl_window_->reconstruction = &reconstruction_manager_.Get(reconstruction_idx);
        opengl_window_->Update();
    }

    void MainWindow::RenderClear() {
        reconstruction_manager_widget_->SelectReconstruction(
                ReconstructionManagerWidget::kNewestReconstructionIdx);
        opengl_window_->Clear();
    }

    void MainWindow::SelectReconstructionIdx(const size_t) {
        RenderSelectedReconstruction();
    }

    size_t MainWindow::SelectedReconstructionIdx() {
        size_t reconstruction_idx =
                reconstruction_manager_widget_->SelectedReconstructionIdx();
        if (reconstruction_idx ==
            ReconstructionManagerWidget::kNewestReconstructionIdx) {
            if (reconstruction_manager_.Size() > 0) {
                reconstruction_idx = reconstruction_manager_.Size() - 1;
            }
        }
        return reconstruction_idx;
    }

    bool MainWindow::HasSelectedReconstruction() {
        const size_t reconstruction_idx =
                reconstruction_manager_widget_->SelectedReconstructionIdx();
        if (reconstruction_idx ==
            ReconstructionManagerWidget::kNewestReconstructionIdx) {
            if (reconstruction_manager_.Size() == 0) {
                return false;
            }
        }
        return true;
    }

    void MainWindow::ShowLog() {
        log_widget_->show();
        log_widget_->raise();
        dock_log_widget_->show();
        dock_log_widget_->raise();
    }

    void MainWindow::UpdateTimer() {
        const int elapsed_time = static_cast<int>(timer_.ElapsedSeconds());
        const int seconds = elapsed_time % 60;
        const int minutes = (elapsed_time / 60) % 60;
        const int hours = (elapsed_time / 3600) % 24;
        const int days = elapsed_time / 86400;
        statusbar_timer_label_->setText(QString().sprintf(
                "Time %02d:%02d:%02d:%02d", days, hours, minutes, seconds));
    }

    void MainWindow::ShowInvalidProjectError() {
        QMessageBox::critical(this, "",
                              tr("You must create a valid project using: <i>File > "
                                         "New project</i> or <i>File > Edit project</i>"));
    }

    void MainWindow::EnableBlockingActions() {
        for (auto& action : blocking_actions_) {
            action->setEnabled(true);
        }
    }

    void MainWindow::DisableBlockingActions() {
        for (auto& action : blocking_actions_) {
            action->setDisabled(true);
        }
    }

    void MainWindow::UpdateWindowTitle() {
        if (*options_.project_path == "") {
            setWindowTitle(QString::fromStdString("BKMAP"));
        } else {
            std::string project_title = *options_.project_path;
            if (project_title.size() > 80) {
                project_title =
                        "..." + project_title.substr(project_title.size() - 77, 77);
            }
            setWindowTitle(QString::fromStdString("BKMAP - " + project_title));
        }
    }

    void MainWindow::WriteAnalysisFile(){


        const std::string analys_path =
                QFileDialog::getSaveFileName(this, tr("Select path to analysis file"), "",
                                             tr("Analysis (*.log)"))
                        .toUtf8()
                        .constData();

        if (analys_path == "") {
            return;
        }

        std::ofstream file(analys_path, std::ios::app);
        CHECK(file.is_open()) << analys_path;
        auto reconstruction =  reconstruction_manager_.Get(SelectedReconstructionIdx());

        file << StringPrintf("Number of images: %d \n", reconstruction.NumImages());
        file << StringPrintf("Number of points: %d \n", reconstruction.NumPoints3D());
        file << StringPrintf("Mean reprojection error: %fpx \n",
                             reconstruction.ComputeMeanReprojectionError());
    }

}