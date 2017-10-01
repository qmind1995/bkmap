set(FOLDER_NAME "estimators")

BKMAP_ADD_LIBRARY(estimators
        absolute_pose.h absolute_pose.cpp
        affine_transform.h affine_transform.cpp
        coordinate_frame.h coordinate_frame.cpp
        essential_matrix.h essential_matrix.cpp
        euclidean_transform.h
        fundamental_matrix.h fundamental_matrix.cpp
        generalized_absolute_pose.h generalized_absolute_pose.cpp
        generalized_absolute_pose_coeffs.h generalized_absolute_pose_coeffs.cpp
        generalized_relative_pose.h generalized_relative_pose.cpp
        homography_matrix.h homography_matrix.cpp
        pose.h pose.cpp
        similarity_transform.h
        translation_transform.h
        triangulation.h triangulation.cpp
        two_view_geometry.h two_view_geometry.cpp
        utils.h utils.cpp
        )