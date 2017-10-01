set(FOLDER_NAME "sfm")

BKMAP_ADD_LIBRARY(sfm
    incremental_mapper.h incremental_mapper.cpp
    incremental_triangulator.h incremental_triangulator.cpp
)
