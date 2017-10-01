//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_RECONSTRUCTION_STATS_WIDGET_H
#define BKMAP_RECONSTRUCTION_STATS_WIDGET_H

#include <QtWidgets>

#include "base/reconstruction.h"

namespace bkmap {

    class ReconstructionStatsWidget : public QWidget {
    public:
        explicit ReconstructionStatsWidget(QWidget* parent);

        void Show(const Reconstruction& reconstruction);

    private:
        void AddStatistic(const QString& header, const QString& content);

        QTableWidget* stats_table_;
    };

}

#endif //BKMAP_RECONSTRUCTION_STATS_WIDGET_H
