cmake_minimum_required(VERSION 3.18)

project(lidar_3d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lidar_3d
    main.cpp
)

add_executable(visualize
    visualize.cpp
)

target_link_libraries(lidar_3d
    PRIVATE
        Open3D::Open3D
        Eigen3::Eigen
)

target_link_libraries(visualize
    PRIVATE
        Open3D::Open3D
)
