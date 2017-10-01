//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_MATCH_MATRIX_WIDGET_H
#define BKMAP_MATCH_MATRIX_WIDGET_H

#include "ui/image_viewer_widget.h"
#include "util/option_manager.h"

namespace bkmap {

// Widget to visualize match matrix.
    class MatchMatrixWidget : public ImageViewerWidget {
    public:
        MatchMatrixWidget(QWidget* parent, OptionManager* options);

        void Show();

    private:
        OptionManager* options_;
    };

}

#endif //BKMAP_MATCH_MATRIX_WIDGET_H
