//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_FEATURE_MATCHING_WIDGET_H
#define BKMAP_FEATURE_MATCHING_WIDGET_H

#include <QtCore>
#include <QtWidgets>

#include "util/misc.h"
#include "util/option_manager.h"

namespace bkmap {

    class FeatureMatchingWidget : public QWidget {
    public:
        FeatureMatchingWidget(QWidget* parent, OptionManager* options);

    private:
        void showEvent(QShowEvent* event);
        void hideEvent(QHideEvent* event);

        QWidget* parent_;
        QTabWidget* tab_widget_;
    };

}

#endif //BKMAP_FEATURE_MATCHING_WIDGET_H
