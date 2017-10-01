set(FOLDER_NAME "retrieval")

BKMAP_ADD_LIBRARY(retrieval
        geometry.h geometry.cpp
        inverted_file.h
        inverted_file_entry.h
        inverted_index.h
        utils.h
        visual_index.h
        vote_and_verify.h vote_and_verify.cpp
        )
