cmake_minimum_required(VERSION 3.10)

project(DataTypes)
set(CMAKE_CXX_STANDARD 17)

find_package(Catch2 3 REQUIRED)

add_library(Vec3 vec3.cpp vec3.hpp)

add_executable(Vec3Test vec3_test.cpp)
target_link_libraries(Vec3Test PUBLIC Vec3 Catch2::Catch2WithMain)

add_library(Edge edge.cpp edge.hpp)
target_link_libraries(Edge PUBLIC Vec3)

add_executable(EdgeTest edge_test.cpp)
target_link_libraries(EdgeTest PUBLIC Edge Catch2::Catch2WithMain)