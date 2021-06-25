set(SOURCE_LIST
    common/loader.h
    common/linalg.h
    common/geom.h
    common/curve.h
    common/screen.h
    common/util.h
    common/scene.h
    backend/interface.h
    backend/cpu/renderer.h
    backend/cpu/renderer.cpp
    backend/gl/renderer.h
    backend/gl/renderer.cpp
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_LIST})

add_library(focg ${SOURCE_LIST})
target_include_directories(focg PUBLIC ..)
