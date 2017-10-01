//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_UNDISTORTION_WIDGET_H
#define BKMAP_UNDISTORTION_WIDGET_H

#include <QtCore>
#include <QtWidgets>

#include "base/undistortion.h"
#include "ui/opengl_window.h"
#include "ui/options_widget.h"
#include "ui/thread_control_widget.h"
#include "util/misc.h"

namespace bkmap {

    class UndistortionWidget : public OptionsWidget {
    public:
        UndistortionWidget(QWidget* parent, const OptionManager* options);

        void Show(const Reconstruction& reconstruction);
        bool IsValid() const;

    private:
        void Undistort();

        const OptionManager* options_;
        const Reconstruction* reconstruction_;

        ThreadControlWidget* thread_control_widget_;

        QComboBox* output_format_;
        UndistortCameraOptions undistortion_options_;
        std::string output_path_;
    };

}

#endif //BKMAP_UNDISTORTION_WIDGET_H
