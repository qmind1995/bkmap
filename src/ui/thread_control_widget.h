//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_THREAD_CONTROL_WIDGET_H
#define BKMAP_THREAD_CONTROL_WIDGET_H

#include <QtCore>
#include <QtWidgets>

#include "util/threading.h"

namespace bkmap {

    class ThreadControlWidget : public QWidget {
    public:
        explicit ThreadControlWidget(QWidget* parent);

        void StartThread(const QString& progress_text, const bool stoppable,
                         Thread* thread);
        void StartFunction(const QString& progress_text,
                           const std::function<void()>& func);

    private:
        QProgressDialog* progress_bar_;
        QAction* destructor_;
        std::unique_ptr<Thread> thread_;
    };

}

#endif //BKMAP_THREAD_CONTROL_WIDGET_H
