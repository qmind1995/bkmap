//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_FEATURE_EXTRACTION_WIDGET_H
#define BKMAP_FEATURE_EXTRACTION_WIDGET_H

#include <QtCore>
#include <QtWidgets>

#include "util/misc.h"
#include "util/option_manager.h"

namespace bkmap {

    class FeatureExtractionWidget : public QWidget {
    public:
        FeatureExtractionWidget(QWidget* parent, OptionManager* options);

    private:
        void showEvent(QShowEvent* event);
        void hideEvent(QHideEvent* event);

        void ReadOptions();
        void WriteOptions();

        QGroupBox* CreateCameraModelBox();

        void SelectCameraModel(const int code);
        void Extract();

        QWidget* parent_;

        OptionManager* options_;

        QComboBox* camera_model_cb_;
        QCheckBox* single_camera_cb_;
        QRadioButton* camera_params_exif_rb_;
        QRadioButton* camera_params_custom_rb_;
        QLabel* camera_params_info_;
        QLineEdit* camera_params_text_;

        std::vector<int> camera_model_ids_;

        QTabWidget* tab_widget_;
    };

}

#endif //BKMAP_FEATURE_EXTRACTION_WIDGET_H
