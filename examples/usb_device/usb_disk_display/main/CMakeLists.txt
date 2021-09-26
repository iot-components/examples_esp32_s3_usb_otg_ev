idf_component_register(
    SRC_DIRS "." "./app/usb_camera" "./app/avi_player" "./app/app_menu" "./app/usb_wireless_disk" "./app/app_manual"#"./app/app_hid_master"
    INCLUDE_DIRS "." "./app" #"~/incubator/iot-component/examples_esp32_s3_usb_otg_ev/examples/usb_host/usb_hid_host/prebuild/components/hid/include"
    #"~/incubator/iot-component/examples_esp32_s3_usb_otg_ev/examples/usb_host/usb_hid_host/prebuild/components/hid/hidapi/hidapi"
    EMBED_FILES "./app/usb_wireless_disk/favicon.ico" "./app/usb_wireless_disk/upload_script.html"
)

# Import the library, specifying a target name and the library path.
# The private dependencies of the library is also specified.

# add_prebuilt_library(libusb "libusb.a"
#              PRIV_REQUIRES driver)

# add_prebuilt_library(hid "libhid.a"
#              PRIV_REQUIRES)

# # `main` calls a function from the library, so link it to `main`
# target_link_libraries(hid PRIVATE libusb)

spiffs_create_partition_image(storage ../spiffs_image FLASH_IN_PROJECT)
