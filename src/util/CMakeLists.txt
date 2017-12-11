set(FOLDER_NAME "util")

BKMAP_ADD_LIBRARY(util
        bitmap.h bitmap.cpp
        camera_specs.h camera_specs.cpp
        logging.h logging.cpp
        math.h math.cpp
        misc.h misc.cpp
        opengl_utils.h opengl_utils.cpp
        option_manager.h option_manager.cpp
        random.h random.cpp
        string.h string.cpp
        threading.h threading.cpp
        timer.h timer.cpp
        version.h version.cc
        )

if(CUDA_ENABLED)
    BKMAP_CUDA_ADD_LIBRARY(util_cuda
            cuda.h cuda.cpp
            cudacc.h cudacc.cpp
            )
endif()
