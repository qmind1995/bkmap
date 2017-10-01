set(FOLDER_NAME "optim")

BKMAP_ADD_LIBRARY(optim
    bundle_adjustment.h bundle_adjustment.cpp
    combination_sampler.h combination_sampler.cpp
    least_absolute_deviations.h least_absolute_deviations.cpp
    progressive_sampler.h progressive_sampler.cpp
    random_sampler.h random_sampler.cpp
    sprt.h sprt.cpp
    support_measurement.h support_measurement.cpp
)