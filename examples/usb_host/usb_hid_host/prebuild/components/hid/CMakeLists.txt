
idf_component_register( SRCS 
               hidapi/libusb/hid.c
               descriptor.c
               libusb_adapter.c
               INCLUDE_DIRS include hidapi/hidapi
               REQUIRES usb pthread)
               
# set_source_files_properties( ${CMAKE_CURRENT_LIST_DIR}/libuvc/src/device.c PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
# set_source_files_properties( ${CMAKE_CURRENT_LIST_DIR}/libuvc/src/stream.c PROPERTIES COMPILE_FLAGS -Wno-unused-variable)

target_compile_definitions(${COMPONENT_LIB} PRIVATE NO_ICONV=1)
# target_include_directories(${COMPONENT_LIB} PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/include/)