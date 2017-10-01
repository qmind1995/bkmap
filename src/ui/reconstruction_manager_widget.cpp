//
// Created by tri on 26/09/2017.
//

#include "ui/reconstruction_manager_widget.h"

namespace bkmap {

    const size_t ReconstructionManagerWidget::kNewestReconstructionIdx =
            std::numeric_limits<size_t>::max();

    ReconstructionManagerWidget::ReconstructionManagerWidget(
            QWidget* parent, const ReconstructionManager* reconstruction_manager)
            : QComboBox(parent), reconstruction_manager_(reconstruction_manager) {
        QFont font;
        font.setPointSize(10);
        setFont(font);
    }

    void ReconstructionManagerWidget::Update() {
        if (view()->isVisible()) {
            return;
        }

        blockSignals(true);

        const int prev_idx = currentIndex() == -1 ? 0 : currentIndex();

        clear();

        addItem("Newest model");

        int max_width = 0;
        for (size_t i = 0; i < reconstruction_manager_->Size(); ++i) {
            const QString item = QString().sprintf(
                    "Model %d (%d images, %d points)", static_cast<int>(i + 1),
                    static_cast<int>(reconstruction_manager_->Get(i).NumRegImages()),
                    static_cast<int>(reconstruction_manager_->Get(i).NumPoints3D()));
            QFontMetrics font_metrics(view()->font());
            max_width = std::max(max_width, font_metrics.width(item));
            addItem(item);
        }

        view()->setMinimumWidth(max_width);

        if (reconstruction_manager_->Size() == 0) {
            setCurrentIndex(0);
        } else {
            setCurrentIndex(prev_idx);
        }

        blockSignals(false);
    }

    size_t ReconstructionManagerWidget::SelectedReconstructionIdx() const {
        if (reconstruction_manager_->Size() == 0) {
            return kNewestReconstructionIdx;
        } else {
            if (currentIndex() == 0) {
                return kNewestReconstructionIdx;
            } else {
                return currentIndex() - 1;
            }
        }
    }

    void ReconstructionManagerWidget::SelectReconstruction(const size_t idx) {
        if (reconstruction_manager_->Size() == 0) {
            blockSignals(true);
            setCurrentIndex(0);
            blockSignals(false);
        } else {
            blockSignals(true);
            setCurrentIndex(idx + 1);
            blockSignals(false);
        }
    }

}
