cmake_minimum_required(VERSION 3.17)
project(hopper)

set(CMAKE_CXX_STANDARD 20)

find_package(towr 1.2 REQUIRED)
find_package(ifopt 2.0.1 REQUIRED)

add_executable(hopper_example hopper_example.cpp)
target_link_libraries(hopper_example towr::towr ifopt::ifopt_ipopt)

add_library(hopper SHARED library.cpp)
target_link_libraries(hopper towr::towr ifopt::ifopt_ipopt pthread)