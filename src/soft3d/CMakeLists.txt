set(SOURCE_LIST
    common/linalg.h
    common/geom.h
    common/curve.h
    common/image.h
    common/util.h
    common/transform.h
    common/scene.h
    common/filter.h
    common/texture.h
    common/threadpool.h
    engine/loader.cpp
    engine/loader.h
    engine/engine.cpp
    engine/engine.h
    backend/interface.h
    backend/rtcpu/renderer.h
    backend/rtcpu/renderer.cpp
    backend/rtgl/renderer.h
    backend/rtgl/renderer.cpp
    backend/zcpu/renderer.h
    backend/zcpu/renderer.cpp
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_LIST})
add_library(soft3d ${SOURCE_LIST})
target_link_libraries(soft3d PRIVATE glfw glad stb_image)
target_include_directories(soft3d PUBLIC .. ${CMAKE_CURRENT_BINARY_DIR}/..)

create_resources(${CMAKE_CURRENT_SOURCE_DIR}/backend/rtgl/shaders ${CMAKE_CURRENT_BINARY_DIR}/backend/rtgl/shaders.h)