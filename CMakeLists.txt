cmake_minimum_required(VERSION 3.3)
project(nuguri3d)

include(EmbedFile.cmake)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 17) 

add_subdirectory(external)
add_subdirectory(src/nuguri3d)
add_subdirectory(src/demo)