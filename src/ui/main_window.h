//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_MAIN_WINDOW_H
#define BKMAP_MAIN_WINDOW_H

#include <QtCore>
#include <QtGui>
#include <QtWidgets>

#include "base/reconstruction.h"
#include "controllers/incremental_mapper.h"
//#include "ui/automatic_reconstruction_widget.h"
#include "ui/bundle_adjustment_widget.h"
#include "ui/database_management_widget.h"
#include "ui/feature_extraction_widget.h"
#include "ui/feature_matching_widget.h"
#include "ui/license_widget.h"
#include "ui/log_widget.h"
#include "ui/match_matrix_widget.h"
#include "ui/opengl_window.h"
#include "ui/project_widget.h"
#include "ui/reconstruction_manager_widget.h"
#include "ui/reconstruction_options_widget.h"
#include "ui/reconstruction_stats_widget.h"
#include "ui/render_options_widget.h"
#include "ui/undistortion_widget.h"
#include "util/bitmap.h"

namespace bkmap {

    class MainWindow : public QMainWindow {
    public:
        explicit MainWindow(const OptionManager& options);

        const ReconstructionManager& GetReconstructionManager() const;

    protected:
        void showEvent(QShowEvent* event);
        void afterShowEvent();
        void closeEvent(QCloseEvent* event);

    private:
        friend class AutomaticReconstructionWidget;
        friend class BundleAdjustmentWidget;
        friend class DenseReconstructionWidget;

        void CreateWidgets();
        void CreateActions();
        void CreateToolbar();
        void CreateStatusbar();
        void CreateControllers();

        void ProjectNew();
        void ExportPlyFile();
        void FeatureExtraction();
        void FeatureMatching();
        void DatabaseManagement();

        void ReconstructionStart();
        void ReconstructionPause();
        void ReconstructionReset();
        void ReconstructionFinish();
        bool ReconstructionOverwrite();


        void Render();
        void RenderNow();
        void RenderToggle();
        void RenderOptions();
        void RenderSelectedReconstruction();
        void RenderClear();

        void SelectReconstructionIdx(const size_t);
        size_t SelectedReconstructionIdx();
        bool HasSelectedReconstruction();
        bool IsSelectedReconstructionValid();

        void ReconstructionStats();
        void MatchMatrix();
        void ShowLog();

        void ShowInvalidProjectError();
        void UpdateTimer();

        void EnableBlockingActions();
        void DisableBlockingActions();

        void UpdateWindowTitle();

        void WriteAnalysisFile();

        void RenderCamera();
        void ShowAnalyze();


        OptionManager options_;

        ReconstructionManager reconstruction_manager_;
        std::unique_ptr<IncrementalMapperController> mapper_controller_;

        Timer timer_;

        OpenGLWindow* opengl_window_;
        ProjectWidget* project_widget_;
        FeatureExtractionWidget* feature_extraction_widget_;
        FeatureMatchingWidget* feature_matching_widget_;
        DatabaseManagementWidget* database_management_widget_;
        RenderOptionsWidget* render_options_widget_;
        LogWidget* log_widget_;
        ReconstructionManagerWidget* reconstruction_manager_widget_;
        ReconstructionStatsWidget* reconstruction_stats_widget_;
        MatchMatrixWidget* match_matrix_widget_;
        ThreadControlWidget* thread_control_widget_;

        QToolBar* file_toolbar_;
        QToolBar* preprocessing_toolbar_;
        QToolBar* reconstruction_toolbar_;

        QDockWidget* dock_log_widget_;

        QTimer* statusbar_timer_;
        QLabel* statusbar_timer_label_;

        QAction* after_show_event_;

        QAction* action_project_new_;
        QAction* action_quit_;

        QAction* action_feature_extraction_;
        QAction* action_feature_matching_;
        QAction* action_database_management_;

        QAction* action_reconstruction_start_;
        QAction* action_reconstruction_pause_;
        QAction* action_reconstruction_reset_;
        QAction* action_reconstruction_finish_;
        QAction* action_render_camera;
        QAction* action_analyze_object;
        QAction* action_export_ply;

        QAction* action_render_;
        QAction* action_render_now_;
        QAction* action_render_toggle_;
        QAction* action_render_reset_view_;
        QAction* action_render_options_;

        QAction* action_reconstruction_stats_;
        QAction* action_match_matrix_;
        QAction* action_log_show_;

        std::vector<QAction*> blocking_actions_;

        // Necessary for OS X to avoid duplicate closeEvents.
        bool window_closed_;
    };

}

#endif //BKMAP_MAIN_WINDOW_H
