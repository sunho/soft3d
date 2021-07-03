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
add_library(nuguri3d ${SOURCE_LIST})
target_link_libraries(nuguri3d PRIVATE glfw glad stb_image)
target_include_directories(nuguri3d PUBLIC .. ${CMAKE_CURRENT_BINARY_DIR}/..)

add_executable(
	linalg-test
	tests/linalg_test.cpp
)
target_link_libraries(linalg-test PRIVATE googletest focg)
add_test(NAME nuguri3d COMMAND linalg-test)

create_resources(${CMAKE_CURRENT_SOURCE_DIR}/backend/rtgl/shaders ${CMAKE_CURRENT_BINARY_DIR}/backend/rtgl/shaders.h)