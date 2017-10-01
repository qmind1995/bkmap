//
// Created by tri on 24/09/2017.
//

#ifndef BKMAP_PROJECTION_H
#define BKMAP_PROJECTION_H

#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "base/camera.h"

namespace bkmap {

// Compose projection matrix from rotation and translation components.
//
// The projection matrix transforms 3D world to image points.
//
// @param qvec           Unit Quaternion rotation coefficients (w, x, y, z).
// @param tvec           3x1 translation vector.
//
// @return               3x4 projection matrix.
    Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Vector4d& qvec,
                                              const Eigen::Vector3d& tvec);

// Compose projection matrix from rotation matrix and translation components).
//
// The projection matrix transforms 3D world to image points.
//
// @param R              3x3 rotation matrix.
// @param t              3x1 translation vector.
//
// @return               3x4 projection matrix.
    Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Matrix3d& R,
                                              const Eigen::Vector3d& t);

// Invert projection matrix, defined as:
//
//    P = [R | t] with R \in SO(3) and t \in R^3
//
// and the inverse projection matrix as:
//
//    P' = [R^T | -R^T t]
//
// @param proj_matrix    3x4 projection matrix.
//
// @return               3x4 inverse projection matrix.
    Eigen::Matrix3x4d InvertProjectionMatrix(const Eigen::Matrix3x4d& proj_matrix);

// Project 3D point to image.
//
// @param points3D         3D world point as 3x1 vector.
// @param proj_matrix      3x4 projection matrix.
// @param camera           Camera used to project to image plane.
//
// @return                 Projected image point.
    Eigen::Vector2d ProjectPointToImage(const Eigen::Vector3d& point3D,
                                        const Eigen::Matrix3x4d& proj_matrix,
                                        const Camera& camera);

// Calculate the reprojection error.
//
// The reprojection error is the Euclidean distance between the observation
// in the image and the projection of the 3D point into the image.
//
// @param points2D         2D image point as 2x1 vector.
// @param points3D         3D world point as 3x1 vector.
// @param proj_matrix      3x4 projection matrix.
// @param camera           Camera used to project to image plane.
//
// @return                 Reprojection error.
    double CalculateReprojectionError(const Eigen::Vector2d& point2D,
                                      const Eigen::Vector3d& point3D,
                                      const Eigen::Matrix3x4d& proj_matrix,
                                      const Camera& camera);

// Calculate the angular error.
//
// The angular error is the angle between the observed viewing ray and the
// actual viewing ray from the camera center to the 3D point.
//
// @param points2D         2D image point as 2x1 vector.
// @param points3D         3D world point as 3x1 vector.
// @param proj_matrix      3x4 projection matrix.
// @param camera           Camera used to project to image plane.
//
// @return                 Angular error.
    double CalculateAngularError(const Eigen::Vector2d& point2D,
                                 const Eigen::Vector3d& point3D,
                                 const Eigen::Matrix3x4d& proj_matrix,
                                 const Camera& camera);

// Calculate angulate error using normalized image points.
//
// The angular error is the angle between the observed viewing ray and the
// actual viewing ray from the camera center to the 3D point.
//
// @param points2D         Normalized 2D image point as 2x1 vector.
// @param points3D         3D world point as 3x1 vector.
// @param proj_matrix      3x4 projection matrix.
//
// @return                 Angular error.
    double CalculateAngularError(const Eigen::Vector2d& point2D,
                                 const Eigen::Vector3d& point3D,
                                 const Eigen::Matrix3x4d& proj_matrix);

// Calculate depth of 3D point with respect to camera.
//
// The depth is defined as the Euclidean distance of a 3D point from the
// camera and is positive if the 3D point is in front and negative if
// behind of the camera.
//
// @param proj_matrix     3x4 projection matrix.
// @param point3D         3D point as 3x1 vector.
//
// @return                Depth of 3D point.
    double CalculateDepth(const Eigen::Matrix3x4d& proj_matrix,
                          const Eigen::Vector3d& point3D);

// Check if 3D point passes cheirality constraint,
// i.e. it lies in front of the camera and not in the image plane.
//
// @param proj_matrix     3x4 projection matrix.
// @param point3D         3D point as 3x1 vector.
//
// @return                True if point lies in front of camera.
    bool HasPointPositiveDepth(const Eigen::Matrix3x4d& proj_matrix,
                               const Eigen::Vector3d& point3D);

}

#endif //BKMAP_PROJECTION_H
