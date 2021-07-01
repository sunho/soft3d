add_executable(focgdemo 3ddemo.h 2ddemo.h main.cpp)
target_link_libraries(focgdemo PRIVATE focg glfw glad)