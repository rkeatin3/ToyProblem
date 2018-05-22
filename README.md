# ToyProblem
## Brief Description
Author: Ryan Keating

This repository contains the C++ translation of a simple Matlab EKF for tracking clock state.

## Dependencies
[Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page): A Template Library for Linear Algebra

On Ubuntu 14.04, it can be install via the command `sudo apt-get install libeigen3-dev`

The CMakeLists.txt file in this repository assumes that Eigen is installed in a default include directory for GCC (like `/usr/include`, the directory to which files will be installed with the above comand).

## How to Build
To build the code, follow the normal steps for building with CMake (from the root of the repository):
1. `$ mkdir build`
2. `$ cd build` 
3. `$ cmake ..`
4. `$ make`

## How to Run
`$ analysis <PathToDatafiles>`