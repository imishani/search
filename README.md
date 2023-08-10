# **Search** algorithms package

#### This package contains implementations of search algorithms

---


### The following algorithms are implemented

---
* **Uninformed Search**
  * _Breadth-First Search_
* **Best-First Search**
  * _Dijkstra's algorithm_
  * _A* search_
  * _Weighted A* search_
  * _ARA* search_ (Anytime Repairing A*)
* **Multi-Agent Path Finding**
  * _Conflict-Based Search_

<u>Note</u>: More algorithms will be added in the near future.

---
### Requirements

---
### The following packages are required to build the package
- OpenCV
- Boost 

---
### Building

---

The package is built using CMake. The following commands can be used to build and install the package
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
The package is designed to be used as a library. In _domain_ directory, you will find two examples that demonstrate how 
to use the library. The examples are in two domains:
- 2D grid search
- Multi-Agent Path Finding (MAPF)

