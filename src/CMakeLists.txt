set(SOURCE_LIST
    focg/loader.h
    focg/geom.h
    focg/curve.h
    focg/raytracer.h
    focg/screen.h
    focg/util.h
    focg/raytracer.cpp
)

add_library(focg ${SOURCE_LIST})
target_include_directories(focg PUBLIC .)
add_subdirectory(demo)
