set(FOLDER_NAME "controllers")

BKMAP_ADD_LIBRARY(controllers
        automatic_reconstruction.h automatic_reconstruction.cpp
        bundle_adjustment.h bundle_adjustment.cpp
        hierarchical_mapper.h hierarchical_mapper.cpp
        incremental_mapper.h incremental_mapper.cpp)
