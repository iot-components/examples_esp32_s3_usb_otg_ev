idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "" "../prebuild/components/hid/include" "../prebuild/components/hid/hidapi/hidapi" )

# Import the library, specifying a target name and the library path.
# The private dependencies of the library is also specified.

add_prebuilt_library(hid "libhid.a"
             PRIV_REQUIRES usb)

# `main` calls a function from the library, so link it to `main`
target_link_libraries(${COMPONENT_LIB} PRIVATE hid)