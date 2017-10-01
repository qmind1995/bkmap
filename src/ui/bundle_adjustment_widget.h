//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_BUNDLE_ADJUSTMENT_WIDGET_H
#define BKMAP_BUNDLE_ADJUSTMENT_WIDGET_H

#include <QtCore>
#include <QtWidgets>

#include "ui/options_widget.h"
#include "ui/thread_control_widget.h"
#include "util/option_manager.h"

namespace bkmap {

    class MainWindow;

    class BundleAdjustmentWidget : public OptionsWidget {
    public:
        BundleAdjustmentWidget(MainWindow* main_window, OptionManager* options);

        void Show(Reconstruction* reconstruction);

    private:
        void Run();
        void Render();

        MainWindow* main_window_;
        OptionManager* options_;
        Reconstruction* reconstruction_;
        ThreadControlWidget* thread_control_widget_;
        QAction* render_action_;
    };

}

#endif //BKMAP_BUNDLE_ADJUSTMENT_WIDGET_H
