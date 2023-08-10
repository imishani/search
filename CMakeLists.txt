cmake_minimum_required(VERSION 3.16.3)
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

add_library(search SHARED
        src/planners/best_first_search.cpp
        src/planners/dijkstra.cpp
        src/planners/astar.cpp
        src/planners/wastar.cpp
        src/planners/arastar.cpp
        src/planners/bfs.cpp
        src/planners/multi_agent/cbs.cpp
        src/common/bfs3d.cpp)

set($(PROJECT_NAME)_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
#set($(PROJECT_NAME)_INCLUDE_DIR include)
set($(PROJECT_NAME)_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")

# support for cmake-config files
configure_file("${PROJECT_NAME}Config.cmake.in" "${PROJECT_NAME}Config.cmake" @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME})

# support for cmake-target files
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
#install(TARGETS search DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(TARGETS search
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

add_executable(run_2d_rob_nav_astar domains/2d_robot_nav/run_2d_astar.cpp)
add_executable(run_2d_rob_nav_dijkstra domains/2d_robot_nav/run_2d_dijkstra.cpp)
add_executable(run_2d_rob_nav_bfs domains/2d_robot_nav/run_2d_bfs.cpp)
add_executable(run_2d_rob_nav_wastar domains/2d_robot_nav/run_2d_wastar.cpp)
add_executable(run_2d_rob_nav_arastar domains/2d_robot_nav/run_2d_arastar.cpp)
add_executable(run_2d_mapf_cbs domains/2d_mapf/run_2d_cbs.cpp)

target_link_libraries(run_2d_rob_nav_astar search ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(run_2d_rob_nav_dijkstra search ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(run_2d_rob_nav_bfs search ${OpenCV_LIBS} ${Boost_LIBRARIES})

target_link_libraries(run_2d_rob_nav_wastar search ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(run_2d_rob_nav_arastar search ${OpenCV_LIBS} ${Boost_LIBRARIES})

target_link_libraries(run_2d_mapf_cbs search ${OpenCV_LIBS} ${Boost_LIBRARIES})

# Add an uninstall target.
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/uninstall.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
        IMMEDIATE @ONLY)
    add_custom_target(uninstall
        COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake")
endif()