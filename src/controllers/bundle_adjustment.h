//
// Created by tri on 25/09/2017.
//

#ifndef BKMAP_BUNDLE_ADJUSTMENT_H
#define BKMAP_BUNDLE_ADJUSTMENT_H

#include "base/reconstruction.h"
#include "util/option_manager.h"
#include "util/threading.h"

namespace bkmap {

// Class that controls the global bundle adjustment procedure.
    class BundleAdjustmentController : public Thread {
    public:
        BundleAdjustmentController(const OptionManager& options,
                                   Reconstruction* reconstruction);

    private:
        void Run();

        const OptionManager options_;
        Reconstruction* reconstruction_;
    };

}

#endif //BKMAP_BUNDLE_ADJUSTMENT_H
