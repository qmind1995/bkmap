//
// Created by tri on 26/09/2017.
//

#include "ui/image_viewer_widget.h"

#include "ui/opengl_window.h"
#include "util/misc.h"

namespace bkmap {

    const double ImageViewerWidget::kZoomFactor = 1.25;

    ImageViewerWidget::ImageViewerWidget(QWidget* parent)
            : QWidget(parent), zoom_scale_(1.0) {
        setWindowFlags(Qt::Window);
        resize(parent->width() - 20, parent->height() - 20);

        QFont font;
        font.setPointSize(10);
        setFont(font);

        grid_layout_ = new QGridLayout(this);
        grid_layout_->setContentsMargins(5, 5, 5, 5);

        image_label_ = new QLabel(this);
        image_scroll_area_ = new QScrollArea(this);
        image_scroll_area_->setWidget(image_label_);
        image_scroll_area_->setSizePolicy(QSizePolicy::Expanding,
                                          QSizePolicy::Expanding);

        grid_layout_->addWidget(image_scroll_area_, 1, 0);
    }

    void ImageViewerWidget::closeEvent(QCloseEvent* event) {
        pixmap_ = QPixmap();
        image_label_->clear();
    }

    void ImageViewerWidget::ShowBitmap(const Bitmap& bitmap, const bool rescale) {
        ShowPixmap(QPixmap::fromImage(BitmapToQImageRGB(bitmap)), rescale);
    }

    void ImageViewerWidget::ShowPixmap(const QPixmap& pixmap, const bool rescale) {
        pixmap_ = pixmap;

        show();
        raise();

        if (rescale) {
            zoom_scale_ = 1.0;

            const double kScrollbarMargin = 5;
            const double scale_x = (image_scroll_area_->width() - kScrollbarMargin) /
                                   static_cast<double>(pixmap_.width());
            const double scale_y = (image_scroll_area_->height() - kScrollbarMargin) /
                                   static_cast<double>(pixmap_.height());
            const double scale = std::min(scale_x, scale_y);

            Rescale(scale);
        } else {
            Rescale(1.0);
        }
    }

    void ImageViewerWidget::ReadAndShow(const std::string& path,
                                        const bool rescale) {
        Bitmap bitmap;
        if (!bitmap.Read(path, true)) {
            std::cerr << "ERROR: Cannot read image at path " << path << std::endl;
        }

        ShowBitmap(bitmap, rescale);
    }

    void ImageViewerWidget::Rescale(const double scale) {
        if (pixmap_.isNull()) {
            return;
        }

        zoom_scale_ *= scale;

        const Qt::TransformationMode transform_mode =
                zoom_scale_ > 1.0 ? Qt::FastTransformation : Qt::SmoothTransformation;
        const int scaled_width =
                static_cast<int>(std::round(zoom_scale_ * pixmap_.width()));
        image_label_->setPixmap(pixmap_.scaledToWidth(scaled_width, transform_mode));
        image_label_->adjustSize();
    }

    void ImageViewerWidget::ZoomIn() { Rescale(kZoomFactor); }

    void ImageViewerWidget::ZoomOut() { Rescale(1.0 / kZoomFactor); }

    FeatureImageViewerWidget::FeatureImageViewerWidget(
            QWidget* parent, const std::string& switch_text)
            : ImageViewerWidget(parent),
              switch_state_(true),
              switch_text_(switch_text) {
    }

    void FeatureImageViewerWidget::ReadAndShowWithKeypoints(
            const std::string& path, const FeatureKeypoints& keypoints,
            const std::vector<char>& tri_mask) {
        Bitmap bitmap;
        if (!bitmap.Read(path, true)) {
            std::cerr << "ERROR: Cannot read image at path " << path << std::endl;
        }

        image1_ = QPixmap::fromImage(BitmapToQImageRGB(bitmap));
        image2_ = image1_;

        const size_t num_tri_keypoints = std::count_if(
                tri_mask.begin(), tri_mask.end(), [](const bool tri) { return tri; });

        FeatureKeypoints keypoints_tri(num_tri_keypoints);
        FeatureKeypoints keypoints_not_tri(keypoints.size() - num_tri_keypoints);
        size_t i_tri = 0;
        size_t i_not_tri = 0;
        for (size_t i = 0; i < tri_mask.size(); ++i) {
            if (tri_mask[i]) {
                keypoints_tri[i_tri] = keypoints[i];
                i_tri += 1;
            } else {
                keypoints_not_tri[i_not_tri] = keypoints[i];
                i_not_tri += 1;
            }
        }

        DrawKeypoints(&image2_, keypoints_tri, Qt::magenta);
        DrawKeypoints(&image2_, keypoints_not_tri, Qt::red);

        if (switch_state_) {
            ShowPixmap(image2_, true);
        } else {
            ShowPixmap(image1_, true);
        }
    }

    void FeatureImageViewerWidget::ReadAndShowWithMatches(
            const std::string& path1, const std::string& path2,
            const FeatureKeypoints& keypoints1, const FeatureKeypoints& keypoints2,
            const FeatureMatches& matches) {
        Bitmap bitmap1;
        Bitmap bitmap2;
        if (!bitmap1.Read(path1, true) || !bitmap2.Read(path2, true)) {
            std::cerr << "ERROR: Cannot read images at paths " << path1 << " and "
            << path2 << std::endl;
            return;
        }

        const auto image1 = QPixmap::fromImage(BitmapToQImageRGB(bitmap1));
        const auto image2 = QPixmap::fromImage(BitmapToQImageRGB(bitmap2));

        image1_ = ShowImagesSideBySide(image1, image2);
        image2_ = DrawMatches(image1, image2, keypoints1, keypoints2, matches);

        if (switch_state_) {
            ShowPixmap(image2_, true);
        } else {
            ShowPixmap(image1_, true);
        }
    }

    void FeatureImageViewerWidget::ShowOrHide() {

    }

    DatabaseImageViewerWidget::DatabaseImageViewerWidget(
            QWidget* parent, OpenGLWindow* opengl_window, OptionManager* options)
            : FeatureImageViewerWidget(parent, "keypoints"),
              opengl_window_(opengl_window),
              options_(options) {
        setWindowTitle("Image information");

        table_widget_ = new QTableWidget(this);
        table_widget_->setColumnCount(2);
        table_widget_->setRowCount(3); // number rows

        QFont font;
        font.setPointSize(10);
        table_widget_->setFont(font);

        table_widget_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

        table_widget_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table_widget_->setSelectionMode(QAbstractItemView::SingleSelection);
        table_widget_->setShowGrid(true);

        table_widget_->horizontalHeader()->setStretchLastSection(true);
        table_widget_->horizontalHeader()->setVisible(false);
        table_widget_->verticalHeader()->setVisible(false);
        table_widget_->verticalHeader()->setDefaultSectionSize(18);
        int row = 0;

        table_widget_->setItem(row, 0, new QTableWidgetItem("num first img's keypoints"));
        num_points2D_item_ = new QTableWidgetItem();
        num_points2D_item_->setForeground(Qt::red);
        table_widget_->setItem(row, 1, num_points2D_item_);
        row += 1;

        table_widget_->setItem(row, 0, new QTableWidgetItem("num images"));
        num_images = new QTableWidgetItem();
        num_images->setForeground(Qt::red);
        table_widget_->setItem(row, 1, num_images);
        row += 1;

        table_widget_->setItem(row, 0, new QTableWidgetItem("total keypoints"));
        total_num_points_2D = new QTableWidgetItem();
        total_num_points_2D->setForeground(Qt::red);
        table_widget_->setItem(row, 1, total_num_points_2D);
        row += 1;

        grid_layout_->addWidget(table_widget_, 0, 0);

    }

    void DatabaseImageViewerWidget::ShowImageAnalyze(OptionManager *option){

        ImageReader::Options reader_options;
        std::string db_path = *option->database_path;
        reader_options.database_path = db_path;
        std::string img_path = *option->image_path;
        reader_options.image_path = img_path;
        Database database(reader_options.database_path);
        ImageReader image_reader(reader_options, &database);

        Camera camera;
        Image image;
        Bitmap bitmap;

        if (image_reader.Next(&camera, &image, &bitmap) ==
            ImageReader::Status::FAILURE) {
            return;
        }
        const auto keypoints = database.ReadKeypoints(image.ImageId());
        const std::vector<char> tri_mask(keypoints.size(), false);

        num_points2D_item_->setText(QString::number(keypoints.size()));

        auto num_keypoints = keypoints.size();
        int num_images_ = 1;
        while (image_reader.NextIndex() < image_reader.NumImages()) {
            Camera camera_test;
            Image image_test;
            Bitmap bitmap_test;
            if (image_reader.Next(&camera_test, &image_test, &bitmap_test) ==
                ImageReader::Status::FAILURE) {
                continue;
            }
            num_images_++;
            const auto keypoints_test = database.ReadKeypoints(image_test.ImageId());
            num_keypoints += keypoints_test.size();
        }

        num_images->setText(QString::number(num_images_));
        total_num_points_2D->setText(QString::number(num_keypoints));

        ReadAndShowWithKeypoints(
                JoinPaths(img_path, image.Name()), keypoints, tri_mask);

    }

    void DatabaseImageViewerWidget::ShowImageWithId(const image_t image_id) {
        if (opengl_window_->images.count(image_id) == 0) {
            return;
        }

        image_id_ = image_id;

        const Image& image = opengl_window_->images.at(image_id);
        const Camera& camera = opengl_window_->cameras.at(image.CameraId());

        image_id_item_->setText(QString::number(image_id));
        camera_id_item_->setText(QString::number(image.CameraId()));
        camera_model_item_->setText(QString::fromStdString(camera.ModelName()));
        camera_params_item_->setText(QString::fromStdString(camera.ParamsToString()));
        qvec_item_->setText(QString::number(image.Qvec(0)) + ", " +
                            QString::number(image.Qvec(1)) + ", " +
                            QString::number(image.Qvec(2)) + ", " +
                            QString::number(image.Qvec(3)));
        tvec_item_->setText(QString::number(image.Tvec(0)) + ", " +
                            QString::number(image.Tvec(1)) + ", " +
                            QString::number(image.Tvec(2)));
        dimensions_item_->setText(QString::number(camera.Width()) + "x" +
                                  QString::number(camera.Height()));
        num_points2D_item_->setText(QString::number(image.NumPoints2D()));

        std::vector<char> tri_mask(image.NumPoints2D());
        for (size_t i = 0; i < image.NumPoints2D(); ++i) {
            tri_mask[i] = image.Point2D(i).HasPoint3D();
        }

        num_points3D_item_->setText(QString::number(image.NumPoints3D()));
        num_obs_item_->setText(QString::number(image.NumObservations()));
        name_item_->setText(QString::fromStdString(image.Name()));

        ResizeTable();

        FeatureKeypoints keypoints(image.NumPoints2D());
        for (point2D_t i = 0; i < image.NumPoints2D(); ++i) {
            keypoints[i].x = static_cast<float>(image.Point2D(i).X());
            keypoints[i].y = static_cast<float>(image.Point2D(i).Y());
        }

        const std::string path = JoinPaths(*options_->image_path, image.Name());
        ReadAndShowWithKeypoints(path, keypoints, tri_mask);
    }

    void DatabaseImageViewerWidget::ResizeTable() {
        // Set fixed table dimensions.
        table_widget_->resizeColumnsToContents();
        int height = table_widget_->horizontalHeader()->height() +
                     2 * table_widget_->frameWidth();
        for (int i = 0; i < table_widget_->rowCount(); i++) {
            height += table_widget_->rowHeight(i);
        }
        table_widget_->setFixedHeight(height);
    }

    void DatabaseImageViewerWidget::DeleteImage() {
        QMessageBox::StandardButton reply = QMessageBox::question(
                this, "", tr("Do you really want to delete this image?"),
                QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes) {
            if (opengl_window_->reconstruction->ExistsImage(image_id_)) {
                opengl_window_->reconstruction->DeRegisterImage(image_id_);
            }
            opengl_window_->Update();
        }
        hide();
    }

}