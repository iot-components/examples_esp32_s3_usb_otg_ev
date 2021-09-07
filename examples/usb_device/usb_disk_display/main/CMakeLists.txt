idf_component_register(
    SRCS "file_server.c" "main.c"
    INCLUDE_DIRS ""
    EMBED_FILES "favicon.ico" "upload_script.html"
)

spiffs_create_partition_image(storage ../spiffs_image FLASH_IN_PROJECT)
