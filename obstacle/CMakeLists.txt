project(Obstacle)
set(CMAKE_CXX_STANDARD 17)

find_package(Catch2 3 REQUIRED)

add_library(Obstacle obstacle.hpp obstacle.cpp)
target_include_directories(Obstacle PUBLIC ../data_types)
target_link_libraries(Obstacle PUBLIC Vec3 Edge ArmDimensions)

add_executable(ObstacleTest obstacle_test.cpp)
target_link_libraries(ObstacleTest PUBLIC Obstacle Catch2::Catch2WithMain)
