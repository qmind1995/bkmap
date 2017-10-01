//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_AUTOMATIC_RECONSTRUCTION_WIDGET_H
#define BKMAP_AUTOMATIC_RECONSTRUCTION_WIDGET_H

#include "controllers/automatic_reconstruction.h"
#include "ui/options_widget.h"
#include "ui/thread_control_widget.h"

namespace bkmap {

    class MainWindow;

    class AutomaticReconstructionWidget : public OptionsWidget {
    public:
        AutomaticReconstructionWidget(MainWindow* main_window);

        void Run();

    private:
        void RenderResult();

        MainWindow* main_window_;
        AutomaticReconstructionController::Options options_;
        ThreadControlWidget* thread_control_widget_;
        QComboBox* data_type_cb_;
        QComboBox* quality_cb_;
        QAction* render_result_;
    };

}

#endif //BKMAP_AUTOMATIC_RECONSTRUCTION_WIDGET_H
