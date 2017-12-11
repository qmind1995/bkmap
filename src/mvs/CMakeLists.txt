set(FOLDER_NAME "mvs")

BKMAP_ADD_LIBRARY(mvs
        consistency_graph.h consistency_graph.cpp
        depth_map.h depth_map.cpp
        fusion.h fusion.cpp
        image.h image.cpp
        meshing.h meshing.cpp
        model.h model.cpp
        normal_map.h normal_map.cpp
        workspace.h workspace.cpp
        )

#COLMAP_ADD_TEST(consistency_graph_test consistency_graph_test.cc)
#COLMAP_ADD_TEST(depth_map_test depth_map_test.cc)
#COLMAP_ADD_TEST(mat_test mat_test.cc)
#COLMAP_ADD_TEST(normal_map_test normal_map_test.cc)

if(CUDA_ENABLED)

    # MSVC/NVCC cannot compile the Boost unit test suite.

endif()
