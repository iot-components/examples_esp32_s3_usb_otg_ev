idf_component_register(
    SRCS "app_main.c" "./app/usb_camera/main/usb_camera_main.c"
    INCLUDE_DIRS "" "./app"
)

spiffs_create_partition_image(storage ../spiffs_image FLASH_IN_PROJECT)
