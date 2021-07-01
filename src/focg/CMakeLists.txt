set(SOURCE_LIST
    common/linalg.h
    common/geom.h
    common/curve.h
    common/image.h
    common/util.h
    common/transform.h
    common/scene.h
    common/filter.h
    engine/loader.h
    engine/engine.h
    engine/engine.cpp
    backend/interface.h
    backend/rtcpu/renderer.h
    backend/rtcpu/renderer.cpp
    backend/rtgl/renderer.h
    backend/rtgl/renderer.cpp
    backend/zcpu/renderer.h
    backend/zcpu/renderer.cpp
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_LIST})
add_library(focg ${SOURCE_LIST})
target_link_libraries(focg PRIVATE glfw glad)
target_include_directories(focg PUBLIC .. ${CMAKE_CURRENT_BINARY_DIR}/..)

add_executable(
	linalg-test
	tests/linalg_test.cpp
)
target_link_libraries(linalg-test PRIVATE googletest focg)
add_test(NAME focg COMMAND linalg-test)

create_resources(${CMAKE_CURRENT_SOURCE_DIR}/backend/rtgl/shaders ${CMAKE_CURRENT_BINARY_DIR}/backend/rtgl/shaders.h)