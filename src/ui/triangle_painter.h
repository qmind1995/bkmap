//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_TRIANGLE_PAINTER_H
#define BKMAP_TRIANGLE_PAINTER_H

#include <QtCore>
#include <QtOpenGL>

#include "ui/point_painter.h"

namespace bkmap {

    class TrianglePainter {
    public:
        TrianglePainter();
        ~TrianglePainter();

        struct Data {
            Data() {}
            Data(const PointPainter::Data& p1, const PointPainter::Data& p2,
                 const PointPainter::Data& p3)
                    : point1(p1), point2(p2), point3(p3) {}

            PointPainter::Data point1;
            PointPainter::Data point2;
            PointPainter::Data point3;
        };

        void Setup();
        void Upload(const std::vector<TrianglePainter::Data>& data);
        void Render(const QMatrix4x4& pmv_matrix);

    private:
        QOpenGLShaderProgram shader_program_;
        QOpenGLVertexArrayObject vao_;
        QOpenGLBuffer vbo_;

        size_t num_geoms_;
    };

}

#endif //BKMAP_TRIANGLE_PAINTER_H
