cmake_minimum_required(VERSION 2.8.3)
project(vis)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")


find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_executable(cloud_toolbox 
    src/cloud_toolbox.cc
    src/bbox.cc 
    src/cloud_toolbox_node.cc
    src/calib.cc)


target_link_libraries(cloud_toolbox
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
