add_executable(demo 3ddemo.h 2ddemo.h main.cpp)
target_link_libraries(demo PRIVATE soft3d glfw glad)