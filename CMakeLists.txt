cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -Wall -Ofast")

project (HAMMER)

#file(src/*)

set (PROJECT_SOURCE_DIR ./src)
set (PROJECT_BINARY_DIR ./bld)
set (EXECUTABLE_OUTPUT_PATH ./bld)
set (LIBRARY_OUTPUT_PATH ./lib)
file(GLOB headers src/hammer/*.hpp)
file(GLOB headers_sub src/hammer/*/*.hpp)

# The version number.
set (Hammer_VERSION_MAJOR 0)
set (Hammer_VERSION_MINOR 1)


# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/hammer/hammerConfig.h.in"
  "${PROJECT_BINARY_DIR}/hammer/hammerConfig.h"
  )

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/${PROJECT_SOURCE_DIR}/cmake")

find_package(Eigen3 
find_package(TBB 4.2 REQUIRED)
find_package(NLopt)
if(NLOPT_FOUND)
	add_definitions(-DUSE_NLOPT)
endif()

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.45.0 REQUIRED COMPONENTS system filesystem ) 

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS}) 
endif()






# add the binary tree to the search path for include files
# so that we will find TutoriConfig.h
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/")
include_directories("${PROJECT_SOURCE_DIR}/../submodules/limbo/src/")


include_directories(${EIGEN3_INCLUDE_DIR})  
include_directories(${TBB_INCLUDE_DIR})  


# add the executable
add_executable(hammerExample ${PROJECT_SOURCE_DIR}/examples/main.cpp)
add_executable(test_hier ${PROJECT_SOURCE_DIR}/examples/test_hier.cpp)

# add libs
link_directories("/usr/local/lib/ ${TBB_LIBRARY}")


target_link_libraries(hammerExample ${TBB_LIBRARIES} ${Boost_LIBRARIES} ${NLOPT_LIBRARIES})
target_link_libraries(test_hier ${TBB_LIBRARIES} ${Boost_LIBRARIES} ${NLOPT_LIBRARIES})
    
