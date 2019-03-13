cmake_minimum_required(VERSION 3.6)

set(PROJECT_NAME simple-texturing)
project(${PROJECT_NAME} LANGUAGES CXX VERSION 0.0.1 DESCRIPTION "simple texture mapping")

set(CMAKE_VERBOSE_MAKEFILE TRUE)

# .lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# .dll and .exe
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin) 

if (WIN32)
# option for Visual Studio
# -EHsc (enable proper Exxeption Handling) needs to avoid C4530
# -Wall is too noisy so that set -W4. 
# https://docs.microsoft.com/en-us/cpp/build/reference/compiler-option-warning-level?view=vs-2017
# "However, for a new project, it may be best to use /W4 in all compilations;
# this will ensure the fewest possible hard-to-find code defects."
set(CMAKE_CXX_FLAGS "-std=c++14 -W4 -EHsc")

else()
# g++ option for *nix
set(CMAKE_CXX_FLAGS  "-std=c++14 -Wall -O2")

endif()

# For OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# third_party directries
set(GLM_INSTALL_DIR "third_party/glm" CACHE PATH "glm installed directory")
message("GLM_INSTALL_DIR: ${GLM_INSTALL_DIR}") 
set(NANORT_INSTALL_DIR "third_party/nanort" CACHE PATH "NanoRT installed directory")
message("NANORT_INSTALL_DIR: ${NANORT_INSTALL_DIR}") 
set(STB_INSTALL_DIR "third_party/stb" CACHE PATH "stb installed directory")
message("STB_INSTALL_DIR: ${STB_INSTALL_DIR}") 
set(TINYOBJLOADER_INSTALL_DIR "third_party/tinyobjloader" CACHE PATH "tinyobjloader installed directory")
message("TINYOBJLOADER_INSTALL_DIR: ${TINYOBJLOADER_INSTALL_DIR}")

# switch for optional libraries
option(SIMPLETEX_USE_STB "Use stb to enable image i/o" ON)
message("SIMPLETEX_USE_STB: ${SIMPLETEX_USE_STB}")
if(SIMPLETEX_USE_STB)
  add_definitions(-DSIMPLETEX_USE_STB)
endif()

option(SIMPLETEX_USE_TINYOBJLOADER "Use tinyobjloader to enable .obj input" ON)
message("SIMPLETEX_USE_TINYOBJLOADER: ${SIMPLETEX_USE_TINYOBJLOADER}")
if(SIMPLETEX_USE_TINYOBJLOADER)
  add_definitions(-DSIMPLETEX_USE_TINYOBJLOADER)
endif()

option(SIMPLETEX_USE_OPENMP "Use OpenMP to enable parallelization" ON)
message("SIMPLETEX_USE_OPENMP: ${SIMPLETEX_USE_OPENMP}")
if(SIMPLETEX_USE_OPENMP)
  add_definitions(-DSIMPLETEX_USE_OPENMP)
endif()

# switch for header-only library implementation
option(SIMPLETEX_TINYOBJLOADER_IMPLEMENTATION "implement tinyobjloader" OFF)
message("SIMPLETEX_TINYOBJLOADER_IMPLEMENTATION: ${SIMPLETEX_TINYOBJLOADER_IMPLEMENTATION}")
if(SIMPLETEX_TINYOBJLOADER_IMPLEMENTATION)
  add_definitions(-DSIMPLETEX_TINYOBJLOADER_IMPLEMENTATION)
endif()

option(SIMPLETEX_STB_IMAGE_IMPLEMENTATION "implement stb image" OFF)
message("SIMPLETEX_STB_IMAGE_IMPLEMENTATION: ${SIMPLETEX_STB_IMAGE_IMPLEMENTATION}")
if(SIMPLETEX_STB_IMAGE_IMPLEMENTATION)
  add_definitions(-DSIMPLETEX_STB_IMAGE_IMPLEMENTATION)
endif()

option(SIMPLETEX_STB_IMAGE_WRITE_IMPLEMENTATION "implement stb image" OFF)
message("SIMPLETEX_STB_IMAGE_WRITE_IMPLEMENTATION: ${SIMPLETEX_STB_IMAGE_WRITE_IMPLEMENTATION}")
if(SIMPLETEX_STB_IMAGE_WRITE_IMPLEMENTATION)
  add_definitions(-DSIMPLETEX_STB_IMAGE_WRITE_IMPLEMENTATION)
endif()

set(PUBLIC_LIB_NAME ${PROJECT_NAME})
add_library(${PUBLIC_LIB_NAME}
    STATIC
    include/common.h
    include/visibility_tester.h
	include/vertex_colorizer.h
    include/common.h
    include/camera.h
    include/pose.h
    include/mesh.h
    include/image.h
    include/log.h

    src/visibility_tester.cc
	src/vertex_colorizer.cc
    src/camera.cc
    src/pose.cc
    src/mesh.cc
    src/image.cc
    src/timer.h
    src/log.cc
)

# set folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

SOURCE_GROUP("Public header files" FILES
    include/common.h
    include/visibility_tester.h
	include/vertex_colorizer.h
    include/camera.h
    include/pose.h
    include/mesh.h
    include/image.h
    include/log.h
)

SOURCE_GROUP("Private header files" FILES
    src/timer.h
)

SOURCE_GROUP("Source files" FILES
    src/visibility_tester.cc
	src/vertex_colorizer.cc
    src/camera.cc
    src/pose.cc
    src/mesh.cc
    src/image.cc
    src/log.cc
)

include_directories(${PUBLIC_LIB_NAME} ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PUBLIC_LIB_NAME} third_party)
include_directories(${PUBLIC_LIB_NAME} ${NANORT_INSTALL_DIR})
include_directories(${PUBLIC_LIB_NAME} ${GLM_INSTALL_DIR})
include_directories(${PUBLIC_LIB_NAME} ${TINYOBJLOADER_INSTALL_DIR})
include_directories(${PUBLIC_LIB_NAME} ${STB_INSTALL_DIR})

set_target_properties(${PUBLIC_LIB_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

set(EXAMPLES_EXE test_fountain)
add_executable(${EXAMPLES_EXE}
    test_fountain.cc)

if (WIN32)
    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${EXAMPLES_EXE})
endif()

target_link_libraries(${EXAMPLES_EXE}
    ${PUBLIC_LIB_NAME})

    