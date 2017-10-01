//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_DENSE_RECONSTRUCTION_WIDGET_H
#define BKMAP_DENSE_RECONSTRUCTION_WIDGET_H

#include <QtCore>
#include <QtWidgets>

#include "mvs/fusion.h"
#include "ui/image_viewer_widget.h"
#include "ui/options_widget.h"
#include "ui/thread_control_widget.h"
#include "util/option_manager.h"

namespace bkmap {

    class MainWindow;

    class DenseReconstructionOptionsWidget : public QWidget {
    public:
        DenseReconstructionOptionsWidget(QWidget* parent, OptionManager* options);
    };

    class DenseReconstructionWidget : public QWidget {
    public:
        DenseReconstructionWidget(MainWindow* main_window, OptionManager* options);

        void Show(Reconstruction* reconstruction);

    private:
        void showEvent(QShowEvent* event);

        void Undistort();
        void Stereo();
        void Fusion();
        void Meshing();

        void SelectWorkspacePath();
        std::string GetWorkspacePath();
        void RefreshWorkspace();

        void WriteFusedPoints();
        void ShowMeshingInfo();

        QWidget* GenerateTableButtonWidget(const std::string& image_name,
                                           const std::string& type);

        MainWindow* main_window_;
        OptionManager* options_;
        Reconstruction* reconstruction_;
        ThreadControlWidget* thread_control_widget_;
        DenseReconstructionOptionsWidget* options_widget_;
        ImageViewerWidget* image_viewer_widget_;
        QLineEdit* workspace_path_text_;
        QTableWidget* table_widget_;
        QPushButton* undistortion_button_;
        QPushButton* stereo_button_;
        QPushButton* fusion_button_;
        QPushButton* meshing_button_;
        QAction* refresh_workspace_action_;
        QAction* write_fused_points_action_;
        QAction* show_meshing_info_action_;

        bool photometric_done_;
        bool geometric_done_;

        std::string images_path_;
        std::string depth_maps_path_;
        std::string normal_maps_path_;

        std::vector<mvs::FusedPoint> fused_points_;
    };

}

#endif //BKMAP_DENSE_RECONSTRUCTION_WIDGET_H
