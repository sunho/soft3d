cmake_minimum_required(VERSION 3.3)
project(soft3d)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 20) 

add_subdirectory(external)
add_subdirectory(src/soft3d)
add_subdirectory(src/demo)