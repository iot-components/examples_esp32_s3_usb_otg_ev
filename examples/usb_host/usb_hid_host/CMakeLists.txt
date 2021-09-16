# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(HID)

add_custom_target(copy_lib)

add_custom_command(
    TARGET copy_lib POST_BUILD
    COMMAND mkdir -p "${PROJECT_DIR}/libs/${IDF_TARGET}"
    COMMAND cp "${BUILD_DIR}/esp-idf/*/*.a" "${PROJECT_DIR}/libs/${IDF_TARGET}"
)

