cmake_minimum_required(VERSION 3.10)

project(DataTypes)
set(CMAKE_CXX_STANDARD 17)

find_package(Catch2 3 REQUIRED)

add_library(Vec3 vec3.hpp vec3.cpp)

add_executable(Vec3Test vec3_test.cpp)
target_link_libraries(Vec3Test PUBLIC Vec3 Catch2::Catch2WithMain)

add_library(Edge edge.hpp edge.cpp)
target_link_libraries(Edge PUBLIC Vec3)

add_executable(EdgeTest edge_test.cpp)
target_link_libraries(EdgeTest PUBLIC Edge Catch2::Catch2WithMain)

add_library(Pose pose.hpp pose.cpp)

add_executable(PoseTest pose_test.cpp)
target_link_libraries(PoseTest PUBLIC Pose Catch2::Catch2WithMain)

add_library(PosePoints pose_points.hpp pose_points.cpp)
target_link_libraries(PosePoints PUBLIC Pose ArmDimensions Vec3 Edge)

add_executable(PosePointsTest pose_points_test.cpp)
target_link_libraries(PosePointsTest PUBLIC PosePoints Catch2::Catch2WithMain)

add_library(ArmDimensions arm_dimensions.hpp arm_dimensions.cpp)

add_executable(ArmDimensionsTest arm_dimensions_test.cpp)
target_link_libraries(ArmDimensionsTest PUBLIC ArmDimensions Catch2::Catch2WithMain)
