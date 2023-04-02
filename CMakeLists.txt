cmake_minimum_required(VERSION 3.23)
project(search)

set(CMAKE_CXX_STANDARD 14)

# add include directories
include_directories(include)

# link opencv libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# link boost libraries
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})

add_library(search
        src/planners/BestFirstSearch.cpp
        src/planners/dijkstra.cpp
        src/planners/AStar.cpp)

add_executable(run_2d_rob_nav domains/2d_robot_nav/run_2d.cpp)

target_link_libraries(run_2d_rob_nav search ${OpenCV_LIBS} ${Boost_LIBRARIES})

