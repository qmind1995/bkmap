//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_IMAGE_VIEWER_WIDGET_H
#define BKMAP_IMAGE_VIEWER_WIDGET_H

#include <QtCore>
#include <QtWidgets>

#include "base/database.h"
#include "base/projection.h"
#include "base/reconstruction.h"
#include "ui/qt_utils.h"
#include "util/option_manager.h"

namespace bkmap {

    class OpenGLWindow;

    class ImageViewerWidget : public QWidget {
    public:
        explicit ImageViewerWidget(QWidget* parent);

        void ShowBitmap(const Bitmap& bitmap, const bool rescale);
        void ShowPixmap(const QPixmap& pixmap, const bool rescale);
        void ReadAndShow(const std::string& path, const bool rescale);

    protected:
        static const double kZoomFactor;

        void closeEvent(QCloseEvent* event);
        void Rescale(const double scale);
        void ZoomIn();
        void ZoomOut();

        QPixmap pixmap_;
        QGridLayout* grid_layout_;
        QHBoxLayout* button_layout_;
        QLabel* image_label_;
        QScrollArea* image_scroll_area_;
        double zoom_scale_;
    };

    class FeatureImageViewerWidget : public ImageViewerWidget {
    public:
        FeatureImageViewerWidget(QWidget* parent, const std::string& switch_text);

        void ReadAndShowWithKeypoints(const std::string& path,
                                      const FeatureKeypoints& keypoints,
                                      const std::vector<char>& tri_mask);

        void ReadAndShowWithMatches(const std::string& path1,
                                    const std::string& path2,
                                    const FeatureKeypoints& keypoints1,
                                    const FeatureKeypoints& keypoints2,
                                    const FeatureMatches& matches);

    protected:
        void ShowOrHide();

        QPixmap image1_;
        QPixmap image2_;
        bool switch_state_;
        QPushButton* switch_button_;
        const std::string switch_text_;
    };

    class DatabaseImageViewerWidget : public FeatureImageViewerWidget {
    public:
        DatabaseImageViewerWidget(QWidget* parent, OpenGLWindow* opengl_window,
                                  OptionManager* options);

        void ShowImageWithId(const image_t image_id);
        void ShowImageAnalyze(OptionManager* option);
    private:
        void ResizeTable();
        void DeleteImage();

        OpenGLWindow* opengl_window_;

        OptionManager* options_;

        QPushButton* delete_button_;

        image_t image_id_;

        QTableWidget* table_widget_;
        QTableWidgetItem* image_id_item_;
        QTableWidgetItem* camera_id_item_;
        QTableWidgetItem* camera_model_item_;
        QTableWidgetItem* camera_params_item_;
        QTableWidgetItem* qvec_item_;
        QTableWidgetItem* tvec_item_;
        QTableWidgetItem* dimensions_item_;
        QTableWidgetItem* num_points2D_item_;
        QTableWidgetItem* num_points3D_item_;
        QTableWidgetItem* num_obs_item_;
        QTableWidgetItem* name_item_;
        QTableWidgetItem* total_num_points_2D;
        QTableWidgetItem* num_images;
    };

}


#endif //BKMAP_IMAGE_VIEWER_WIDGET_H
