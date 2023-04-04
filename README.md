# **Search** algorithms package

---
### This package contains implementations of search algorithms.

## The following algorithms are implemented

---
### - Best-first search
### - Weighted A* search
### - A* search
### - Dijkstra's algorithm
### - Greedy best-first search
### Note: More algorithms will be added in the near future.

---
## Requirements

---
### The following packages are required to build the package
### - OpenCV
### - Boost 

---
## Building

---

### The package is built using CMake. The following commands can be used to build and install the package
~~~ 
cd search
mkdir build
cd build
cmake ..
make
sudo make install
~~~

## Usage

-----
### The package is designed to be used as a library. In _domain_ directory, you will find an example that 
### uses the A* search algorithm to find the shortest path between two points in a 2D grid.
