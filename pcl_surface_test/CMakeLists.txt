# Plese, visit http://www.cmake.org/Wiki/CMake_Useful_Variables to understand CMake variables

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCL_FILTERS_TEST)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
SET(CMAKE_BUILD_TYPE Debug)

#set the default path for built executables to the "bin" directory
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
# include the local "include" folder
include_directories(${PCL_FILTERS_TEST_SOURCE_DIR}/include)

# PCL
# TODO:find a better way to include this header files
if(APPLE)
    include_directories (/usr/include/c++/4.2.1) 
else()
    set(PCL_DIR "/home/carlos/src/pcl-1.6.0-build")
endif(APPLE)

find_package(PCL 1.6 REQUIRED COMPONENTS common io filters keypoints features surface kdtree search)
if (PCL_FOUND) 
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  include_directories(${PCL_INCLUDE_DIRS})

  #message("PCL INCLUDE DIRS: ${PCL_INCLUDE_DIRS}")
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  add_library(filtered_clouds_lib src/pcl_filters_utils.cpp)
  target_link_libraries(filtered_clouds_lib ${PCL_LIBRARIES})
  
  add_executable(pcd_write_test src/pcd_write.cpp)
  target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

  add_executable(test_marching_cubes
                              src/test_marching_cubes.cpp)
  target_link_libraries(test_marching_cubes 
                          filtered_clouds_lib 
  						)

  add_executable(test_greedy_mesh
                            src/greedy_projection.cpp)
  target_link_libraries(test_greedy_mesh 
			    filtered_clouds_lib
						)
else()
  message("PCL not found. Disabling pcd2schematic build.")
endif (PCL_FOUND)
