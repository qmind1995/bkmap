//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_QT_UTILS_H
#define BKMAP_QT_UTILS_H

#include <Eigen/Core>

#include <QtCore>
#include <QtOpenGL>

#include "base/feature.h"
#include "util/bitmap.h"
#include "util/types.h"

namespace bkmap {

    Eigen::Matrix4f QMatrixToEigen(const QMatrix4x4& matrix);

    QMatrix4x4 EigenToQMatrix(const Eigen::Matrix4f& matrix);

    QImage BitmapToQImageRGB(const Bitmap& bitmap);

    void DrawKeypoints(QPixmap* image, const FeatureKeypoints& points,
                       const QColor& color = Qt::red);

    QPixmap ShowImagesSideBySide(const QPixmap& image1, const QPixmap& image2);

    QPixmap DrawMatches(const QPixmap& image1, const QPixmap& image2,
                        const FeatureKeypoints& points1,
                        const FeatureKeypoints& points2,
                        const FeatureMatches& matches,
                        const QColor& keypoints_color = Qt::red);

}

#endif //BKMAP_QT_UTILS_H
