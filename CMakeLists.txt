cmake_minimum_required(VERSION 3.23)
project(search)

set(CMAKE_CXX_STANDARD 14)

# add include directories
include_directories(include)

add_library(search
        src/planners/BestFirstSearch.cpp)
