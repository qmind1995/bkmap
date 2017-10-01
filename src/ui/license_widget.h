//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_LICENSE_WIDGET_H
#define BKMAP_LICENSE_WIDGET_H

#include <QtWidgets>

namespace bkmap {

    class LicenseWidget : public QTextEdit {
    public:
        explicit LicenseWidget(QWidget* parent);

    private:
        QString GetCOLMAPLicense() const;
        QString GetFLANNLicense() const;
        QString GetGraclusLicense() const;
        QString GetLSDLicense() const;
        QString GetPBALicense() const;
        QString GetPoissonReconLicense() const;
        QString GetSiftGPULicense() const;
        QString GetSQLiteLicense() const;
        QString GetVLFeatLicense() const;
    };

}

#endif //BKMAP_LICENSE_WIDGET_H
