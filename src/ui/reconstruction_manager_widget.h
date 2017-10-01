//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_RECONSTRUCTION_MANAGER_WIDGET_H
#define BKMAP_RECONSTRUCTION_MANAGER_WIDGET_H

#include <QtWidgets>

#include "base/reconstruction_manager.h"

namespace bkmap {

    class ReconstructionManagerWidget : public QComboBox {
    public:
        const static size_t kNewestReconstructionIdx;

        ReconstructionManagerWidget(
                QWidget* parent, const ReconstructionManager* reconstruction_manager);

        void Update();

        size_t SelectedReconstructionIdx() const;
        void SelectReconstruction(const size_t idx);

    private:
        const ReconstructionManager* reconstruction_manager_;
    };

}

#endif //BKMAP_RECONSTRUCTION_MANAGER_WIDGET_H
