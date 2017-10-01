//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_POINT_VIEWER_WIDGET_H
#define BKMAP_POINT_VIEWER_WIDGET_H

#include <QtCore>
#include <QtWidgets>

#include "base/reconstruction.h"
#include "util/option_manager.h"

namespace bkmap {

    class OpenGLWindow;

    class PointViewerWidget : public QWidget {
    public:
        PointViewerWidget(QWidget* parent, OpenGLWindow* opengl_window,
                          OptionManager* option);

        void Show(const point3D_t point3D_id);

    private:
        void closeEvent(QCloseEvent* event);

        void ClearLocations();
        void UpdateImages();
        void ZoomIn();
        void ZoomOut();
        void Delete();

        OpenGLWindow* opengl_window_;

        OptionManager* options_;

        QPushButton* delete_button_;

        point3D_t point3D_id_;

        QTableWidget* location_table_;
        std::vector<QPixmap> location_pixmaps_;
        std::vector<QLabel*> location_labels_;
        std::vector<image_t> image_ids_;
        std::vector<double> reproj_errors_;

        QPushButton* zoom_in_button_;
        QPushButton* zoom_out_button_;

        double zoom_;
    };

}

#endif //BKMAP_POINT_VIEWER_WIDGET_H
