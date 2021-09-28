# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(PROJECT_VER "0.1.0.0")
set(EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS} $ENV{IDF_PATH}/examples/common_components/qrcode)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_project)

add_custom_target(copy_test_bin)

add_custom_command(
    TARGET copy_test_bin POST_BUILD
    COMMAND cp "${BUILD_DIR}/*.bin" "${PROJECT_DIR}/../usb_device/usb_disk_display/"
)