//
// Created by tri on 21/09/2017.
//

#ifndef BKMAP_OPENGL_UTILS_H
#define BKMAP_OPENGL_UTILS_H

#include <QAction>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QThread>
#include <QWaitCondition>

#include "util/threading.h"

namespace bkmap {

#ifdef DEBUG
#define glDebugLog() glError(__FILE__, __LINE__)
#else
#define glDebugLog()
#endif

// This class manages a thread-safe OpenGL context. Note that this class must be
// instantiated in the main Qt thread, since an OpenGL context must be created
// in it. The context can then be made current in any other thread.
    class OpenGLContextManager : public QObject {
    public:
        OpenGLContextManager();

        // Make the OpenGL context available by moving it from the thread where it was
        // created to the current thread and making it current.
        void MakeCurrent();

        // Check whether the machine has OpenGL and we can create the context.
        static bool HasOpenGL();

    private:
        QOffscreenSurface surface_;
        QOpenGLContext context_;
        QThread* parent_thread_;
        QThread* current_thread_;
        QAction* make_current_action_;
    };

// Run and wait for the thread, that uses the OpenGLContextManager, e.g.:
//
//    class TestThread : public Thread {
//     private:
//      void Run() { opengl_context_.MakeCurrent(); }
//      OpenGLContextManager opengl_context_;
//    };
//    QApplication app(argc, argv);
//    TestThread thread;
//    RunThreadWithOpenGLContext(&thread);
//
    void RunThreadWithOpenGLContext(Thread* thread);

// Get the OpenGL errors and print them to stderr.
    void GLError(const char* file, const int line);

}

#endif //BKMAP_OPENGL_UTILS_H
