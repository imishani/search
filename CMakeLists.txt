cmake_minimum_required(VERSION 3.23)
project(search)

set(CMAKE_CXX_STANDARD 14)

#function(my_export_target _target _include_dir)
#    file(
#            WRITE "${CMAKE_CURRENT_BINARY_DIR}/${_target}Config.cmake"
#            "
#            include(\"\$\{CMAKE_CURRENT_LIST_DIR\}/${_target}Targets.cmake\")
#            set_property(
#                TARGET ${_target}
#                APPEND PROPERTY
#                    INTERFACE_INCLUDE_DIRECTORIES \"${_include_dir}\"
#            )
#        "
#    )
#
#    export(TARGETS ${_target} FILE "${CMAKE_CURRENT_BINARY_DIR}/${_target}Targets.cmake")
#
#    # NOTE: The following call can pollute your PC's CMake package registry
#    #       See comments/alternatives below
#    export(PACKAGE ${_target})
#endfunction(my_export_target)

set($(PROJECT_NAME)_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set($(PROJECT_NAME)_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

# add include directories
include_directories(include)

# link opencv libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# link boost libraries
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})

add_library(search SHARED
        src/planners/BestFirstSearch.cpp
        src/planners/dijkstra.cpp
        src/planners/AStar.cpp)

add_executable(run_2d_rob_nav domains/2d_robot_nav/run_2d.cpp)

target_link_libraries(run_2d_rob_nav search ${OpenCV_LIBS} ${Boost_LIBRARIES})

