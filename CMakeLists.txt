cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -Wall -Ofast")

project (HAMMER)

#file(src/*)

set (PROJECT_SOURCE_DIR ./src)
set (PROJECT_BINARY_DIR ./bld)
set (EXECUTABLE_OUTPUT_PATH ./bld)
set (LIBRARY_OUTPUT_PATH ./lib)


# The version number.
set (Hammer_VERSION_MAJOR 0)
set (Hammer_VERSION_MINOR 1)


# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/hammer/hammerConfig.h.in"
  "${PROJECT_BINARY_DIR}/hammer/hammerConfig.h"
  )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED) 


# add the binary tree to the search path for include files
# so that we will find TutoriConfig.h
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/")


include_directories(${EIGEN3_INCLUDE_DIR})  
include_directories("./submodules/limbo/src/")

# add the executable

add_executable(hammerExample ${PROJECT_SOURCE_DIR}/examples/main.cpp)

# add libs
 link_directories("/usr/local/opt/tbb/lib/,/usr/local/lib/")


target_link_libraries(hammerExample tbb boost_system boost_filesystem)
