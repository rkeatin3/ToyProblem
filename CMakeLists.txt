cmake_minimum_required (VERSION 3.0)

project(ToyProblem)

add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Og -ggdb")

add_executable(analysis Analysis.cpp SimpleEKF.cpp ClockStateModel.cpp ClockMeasurementModel.cpp Parser.cpp)
