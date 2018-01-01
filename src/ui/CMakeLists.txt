set(FOLDER_NAME "ui")

set(BKMAP_QT_MODULES Core OpenGL)

qt5_add_resources(QT_RESOURCES resources.qrc)

BKMAP_ADD_LIBRARY(ui
        bundle_adjustment_widget.h bundle_adjustment_widget.cpp
        colormaps.h colormaps.cpp
        feature_extraction_widget.h feature_extraction_widget.cpp
        feature_matching_widget.h feature_matching_widget.cpp
        image_viewer_widget.h image_viewer_widget.cpp
        line_painter.h line_painter.cpp
        log_widget.h log_widget.cpp
        main_window.h main_window.cpp
        opengl_window.h opengl_window.cpp
        options_widget.h options_widget.cpp
        point_painter.h point_painter.cpp
        point_viewer_widget.h point_viewer_widget.cpp
        project_widget.h project_widget.cpp
        qt_utils.h qt_utils.cpp
        reconstruction_manager_widget.h reconstruction_manager_widget.cpp
        render_options.h render_options.cpp
        thread_control_widget.h thread_control_widget.cpp
        triangle_painter.h triangle_painter.cpp
        ${QT_RESOURCES}
        )
