project(Map)
set(CMAKE_CXX_STANDARD 17)

find_package(Catch2 3 REQUIRED)

add_library(Node node.hpp node.cpp)
target_include_directories(Node PUBLIC ../data_types)
target_link_libraries(Node PUBLIC Pose)

add_executable(NodeTest node_test.cpp)
target_link_libraries(NodeTest PUBLIC Node Catch2::Catch2WithMain)

add_library(NodeComparator node_comparator.hpp node_comparator.cpp)
target_link_libraries(NodeComparator PUBLIC Node)

add_library(Map map.hpp map.cpp)
target_include_directories(Map PUBLIC ../obstacle)
target_link_libraries(Map PUBLIC Node NodeComparator Obstacle)

add_executable(MapTest map_test.cpp)
target_include_directories(MapTest PUBLIC ../obstacle ../data_types)
target_link_libraries(MapTest PUBLIC Map Obstacle ArmDimensions Catch2::Catch2WithMain)