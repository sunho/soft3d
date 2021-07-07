set(SOURCE_LIST
    scene/scene.h
    scene/bvhtree.h
    scene/geom.h
    math/linalg.h
    math/curve.h
    math/transform.h
    image/image.h
    image/filter.h
    image/texture.h
    common/util.h
    common/threadpool.h
    common/idresource.h
    runtime/loader.cpp
    runtime/loader.h
    runtime/runtime.cpp
    runtime/runtime.h
    renderer/interface.h
    renderer/rtcpu/rtcpu.h
    renderer/rtcpu/rtcpu.cpp
    renderer/zcpu/zcpu.h
    renderer/zcpu/zcpu.cpp
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_LIST})
add_library(soft3d ${SOURCE_LIST})
target_link_libraries(soft3d PRIVATE glfw glad stb_image)
target_include_directories(soft3d PUBLIC .. ${CMAKE_CURRENT_BINARY_DIR}/..)