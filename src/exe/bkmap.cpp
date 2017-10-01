//
// Created by tri on 23/09/2017.
//

#include <QtGui>

#include "ui/main_window.h"

using namespace bkmap;

int main(int argc, char** argv) {
    InitializeGlog(argv);

    OptionManager options;

    if (argc > 1) {
        options.AddAllOptions();
        options.Parse(argc, argv);
    }

    Q_INIT_RESOURCE(resources);

    QApplication app(argc, argv);

    MainWindow main_window(options);
    main_window.show();

    return app.exec();
}