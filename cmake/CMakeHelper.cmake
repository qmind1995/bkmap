if(POLICY CMP0043)
    cmake_policy(SET CMP0043 OLD)
endif()
if(POLICY CMP0054)
    cmake_policy(SET CMP0054 OLD)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(IS_MSVC TRUE)
endif()
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(IS_GNU TRUE)
endif()
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(IS_CLANG TRUE)
endif()

string(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWER)
if(CMAKE_BUILD_TYPE_LOWER STREQUAL "debug"
   OR CMAKE_BUILD_TYPE_LOWER STREQUAL "relwithdebinfo")
    set(IS_DEBUG TRUE)
endif()

# Enable solution folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_TARGETS_ROOT_FOLDER "cmake")
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER
    ${CMAKE_TARGETS_ROOT_FOLDER})
set(BKMAP_TARGETS_ROOT_FOLDER "bkmap_targets")
set(BKMAP_SRC_ROOT_FOLDER "bkmap_src")

# Remove the default warning level set by CMake so that later code can allow
# the user to specify a custom warning level.
set(REMOVED_WARNING_LEVEL FALSE)
if(IS_MSVC)
    # CXX Flags
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    elseif(CMAKE_CXX_FLAGS MATCHES "/Wall")
        string(REGEX REPLACE "/Wall" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    endif()
    # C Flags
    if(CMAKE_C_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    elseif(CMAKE_C_FLAGS MATCHES "/Wall")
        string(REGEX REPLACE "/Wall" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    endif()
elseif(IS_GNU OR IS_CLANG)
    # CXX Flags
    if(CMAKE_CXX_FLAGS MATCHES "-Wall")
        string(REGEX REPLACE "-Wall" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    endif()
    # C Flags
    if(CMAKE_C_FLAGS MATCHES "-Wall")
        string(REGEX REPLACE "-Wall" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    endif()
else()
    message(WARNING "Unsupported compiler. Please update CMakeLists.txt")
endif()
if(REMOVED_WARNING_LEVEL)
    message("Removed warning level from default CMAKE_CXX_FLAGS.")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} CACHE STRING
        "Flags used by the compiler during all build types." FORCE)
    set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} CACHE STRING
        "Flags used by the compiler during all build types." FORCE)
endif()

# Construct a list of the possible warning levels that the user can set.
set(COMPILER_DEFAULT_WARNING_LEVEL "<compiler default>")
if(IS_MSVC)
    set(WARNING_LEVEL_OPTIONS
        ${COMPILER_DEFAULT_WARNING_LEVEL}
        /W0 /W1 /W2 /W3 /W4 /Wall)
elseif(IS_GNU OR IS_CLANG)
    set(WARNING_LEVEL_OPTIONS
        ${COMPILER_DEFAULT_WARNING_LEVEL}
        -w -Wall "-Wall -pedantic" "-Wall -pedantic -Wextra -Wno-long-long")
else()
    message(WARNING "Unsupported compiler. Please update CMakeLists.txt")
endif()

# Set the default warning level.
if(IS_MSVC)
    set(BKMAP_WARNING_LEVEL "/W3" CACHE STRING
        "Default compiler warning level.")
elseif(IS_GNU OR IS_CLANG)
    set(BKMAP_WARNING_LEVEL "-Wall -pedantic" CACHE STRING
        "Default compiler warning level.")
else()
    message(WARNING "Unsupported compiler. Please update CMakeLists.txt")
endif()
# Allow the user to change the default warning level using a drop-down list in
# the CMake GUI.
set_property(CACHE BKMAP_WARNING_LEVEL PROPERTY STRINGS
    ${WARNING_LEVEL_OPTIONS})

# Set the default warning level for 3rd-party targets.
if(IS_MSVC)
    set(BKMAP_WARNING_LEVEL_THIRD_PARTY "/W0" CACHE STRING
        "Default compiler warning level for 3rd-party targets.")
elseif(IS_GNU OR IS_CLANG)
    set(BKMAP_WARNING_LEVEL_THIRD_PARTY "-w" CACHE STRING
        "Default compiler warning level for 3rd-party targets.")
else()
    message(WARNING "Unsupported compiler. Please update CMakeLists.txt")
endif()
# Allow the user to change the default warning level for 3rd-party targets
# using a drop-down list in the CMake GUI.
set_property(CACHE BKMAP_WARNING_LEVEL_THIRD_PARTY PROPERTY STRINGS
    ${WARNING_LEVEL_OPTIONS})

# Macro used to define that all following targets should be treated as
# 3rd-party targets.
macro(BKMAP_SET_THIRD_PARTY_FOLDER)
    set(BKMAP_THIRD_PARTY_FOLDER TRUE)
endmacro(BKMAP_SET_THIRD_PARTY_FOLDER)

# Macro used to define that all following targets should not be treated as
# 3rd-party targets.
macro(BKMAP_UNSET_THIRD_PARTY_FOLDER)
    set(BKMAP_THIRD_PARTY_FOLDER FALSE)
endmacro(BKMAP_UNSET_THIRD_PARTY_FOLDER)

# Macro to help provide replacements to the normal add_library(),
# add_executable(), etc. commands.
macro(BKMAP_ADD_TARGET_HELPER TARGET_NAME)
    # Set the name of the folder that will contain this target in the GUI.
    # It is assumed that FOLDER_NAME has already been defined.
    set_target_properties(${TARGET_NAME} PROPERTIES FOLDER
        ${BKMAP_TARGETS_ROOT_FOLDER}/${FOLDER_NAME})

    # Get the warning level to use for this target.
    if(BKMAP_THIRD_PARTY_FOLDER)
        set(CURRENT_WARNING_LEVEL ${BKMAP_WARNING_LEVEL_THIRD_PARTY})
    else()
        set(CURRENT_WARNING_LEVEL ${BKMAP_WARNING_LEVEL})
    endif()

    # Only update the target's compile options if a valid warning level has
    # been selected.
    if(NOT ${CURRENT_WARNING_LEVEL} STREQUAL ${COMPILER_DEFAULT_WARNING_LEVEL})
        # Get the current compile options.
        get_target_property(CURRENT_COMPILE_OPTIONS ${TARGET_NAME} COMPILE_OPTIONS)

        # If compile options have not already been set for this target, set them to
        # a default value.
        if(NOT CURRENT_COMPILE_OPTIONS)
            set(CURRENT_COMPILE_OPTIONS "")
        endif()

        separate_arguments(CURRENT_WARNING_LEVEL)

        # If the warning level consists of multiple tokens, add each token to the
        # current list of compile options.
        foreach(WARNING_LEVEL ${CURRENT_WARNING_LEVEL})
            list(APPEND CURRENT_COMPILE_OPTIONS ${WARNING_LEVEL})
        endforeach()
        unset(WARNING_LEVEL)
        unset(CURRENT_WARNING_LEVEL)

        # Set the target's updated compile options.
        set_target_properties(${TARGET_NAME} PROPERTIES
            COMPILE_OPTIONS "${CURRENT_COMPILE_OPTIONS}")
        unset(CURRENT_COMPILE_OPTIONS)
    endif()
endmacro(BKMAP_ADD_TARGET_HELPER)

# Replacement for the normal add_library() command. The syntax remains the same
# in that the first argument is the target name, and the following arguments
# are the source files to use when building the target.
macro(BKMAP_ADD_LIBRARY TARGET_NAME)
    # ${ARGN} will store the list of source files passed to this function.
    add_library(${TARGET_NAME} ${ARGN})
    qt5_use_modules(${TARGET_NAME} ${BKMAP_QT_MODULES})
    BKMAP_ADD_TARGET_HELPER(${TARGET_NAME})
endmacro(BKMAP_ADD_LIBRARY)

# Replacement for the normal cuda_add_library() command. The syntax remains the
# same in that the first argument is the target name, and the following
# arguments are the source files to use when building the target.
macro(BKMAP_CUDA_ADD_LIBRARY TARGET_NAME)
    # ${ARGN} will store the list of source files passed to this function.
    cuda_add_library(${TARGET_NAME} ${ARGN})
    BKMAP_ADD_TARGET_HELPER(${TARGET_NAME})
endmacro(BKMAP_CUDA_ADD_LIBRARY)

# Replacement for the normal add_executable() command. The syntax remains the
# same in that the first argument is the target name, and the following
# arguments are the source files to use when building the target.
macro(BKMAP_ADD_EXECUTABLE TARGET_NAME)
    # ${ARGN} will store the list of source files passed to this function.
    add_executable(${TARGET_NAME} ${ARGN})
    target_link_libraries(${TARGET_NAME} ${BKMAP_LIBRARIES})
    qt5_use_modules(${TARGET_NAME} ${BKMAP_QT_MODULES})
    BKMAP_ADD_TARGET_HELPER(${TARGET_NAME})
    install(TARGETS ${TARGET_NAME} DESTINATION bin/)
endmacro(BKMAP_ADD_EXECUTABLE)

# Wrapper for test executables


# Wrapper for CUDA test executables

