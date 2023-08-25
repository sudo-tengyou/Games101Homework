cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)
add_subdirectory(transformation)
add_executable(Transformation main.cpp)

target_link_libraries(Transformation Transform)
