set(FOLDER_NAME "base")

BKMAP_ADD_LIBRARY(base
    camera.h camera.cpp
    camera_database.h camera_database.cpp
    camera_models.h camera_models.cpp
    camera_rig.h camera_rig.cpp
    database.h database.cpp
    database_cache.h database_cache.cpp
    essential_matrix.h essential_matrix.cpp
    feature.h feature.cpp
    feature_extraction.h feature_extraction.cpp
    feature_matching.h feature_matching.cpp
    gps.h gps.cpp
    graph_cut.h graph_cut.cpp
    homography_matrix.h homography_matrix.cpp
    image.h image.cpp
    image_reader.h image_reader.cpp
    line.h line.cpp
    point2d.h point2d.cpp
    point3d.h point3d.cpp
    polynomial.h polynomial.cpp
    pose.h pose.cpp
    projection.h projection.cpp
    reconstruction.h reconstruction.cpp
    reconstruction_manager.h reconstruction_manager.cpp
    scene_clustering.h scene_clustering.cpp
    scene_graph.h scene_graph.cpp
    similarity_transform.h similarity_transform.cpp
    track.h track.cpp
    triangulation.h triangulation.cpp
    undistortion.h undistortion.cpp
    visibility_pyramid.h visibility_pyramid.cpp
    warp.h warp.cpp
)
