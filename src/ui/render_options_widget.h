//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_RENDER_OPTIONS_WIDGET_H
#define BKMAP_RENDER_OPTIONS_WIDGET_H

#include <QtCore>
#include <QtWidgets>

#include "sfm/incremental_mapper.h"
#include "ui/opengl_window.h"
#include "ui/options_widget.h"

namespace bkmap {

    class RenderOptionsWidget : public OptionsWidget {
    public:
        RenderOptionsWidget(QWidget* parent, OptionManager* options,
                            OpenGLWindow* opengl_window);

        size_t counter;
        bool automatic_update;

        QAction* action_render_now;

    private:
        void closeEvent(QCloseEvent* event);

        void Apply();
        void ApplyProjection();
        void ApplyColormap();
        void ApplyBackgroundColor();

        void SelectBackgroundColor();

        void IncreasePointSize();
        void DecreasePointSize();
        void IncreaseCameraSize();
        void DecreaseCameraSize();

        OptionManager* options_;
        OpenGLWindow* opengl_window_;

        QComboBox* projection_cb_;
        QComboBox* point3D_colormap_cb_;
        double point3D_colormap_scale_;
        double point3D_colormap_min_q_;
        double point3D_colormap_max_q_;
        QDoubleSpinBox* bg_red_spinbox_;
        QDoubleSpinBox* bg_green_spinbox_;
        QDoubleSpinBox* bg_blue_spinbox_;
        double bg_color_[3];
    };

}

#endif //BKMAP_RENDER_OPTIONS_WIDGET_H
