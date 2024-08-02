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
  * _Experience-Accelerated Weighted A* search_ (EAWA*)
  * _Focal Search_
  * _Focal Weighted A* search_ (FWA*)
  * _Experience-Accelerated Focal Weighted A* search_ (EAFWA*)
  * **Experimental**: _Experience-Graphs search_ (E=Graphs)
* **Sampling-Based Search**
  * _dRRT_
* **Multi-Agent Path Finding**
  * **Conflict-Based Search**
    * _Conflict-Based Search_ (CBS)
    * _Enhanced Conflict-Based Search_ (ECBS)
    * _Conflict-Based MP Search_ (CBS-MP)
    * _Enhanced Conflict-Based MP Search_ (ECBS-MP)
    * _Experience-Accelerated Conflict-Based Search_ (xCBS)
    * _Experience-Accelerated Enhanced Conflict-Based Search_ (xECBS)
  * Prioritized Planning
    * _Prioritized Planning_ (PP)
* **Parallel Search**
  * _Parallel A* with Slow Expansion_ (PA*SE)

<u>Note</u>: More algorithms will be added in the near future.

---
### Requirements

---
### The following packages are required to build the library
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
-----  
Run the following to see an example of 2D grid search in action!
~~~
cd build
./run_2d_rob_nav_wastar 1 50 1
~~~

