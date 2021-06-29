
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_types.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "assert.h"
#include "bsp_esp32_s3_usb_otg_ev.h"
#include "tusb_msc.h"

static sdmmc_card_t *mount_card = NULL;
static const char *TAG = "usb_msc_demo";

#ifdef CONFIG_USE_EXTERNAL_SDCARD
#define BOARD_SDCARD_MOSI_PIN CONFIG_SDCARD_MOSI_PIN
#define BOARD_SDCARD_MISO_PIN CONFIG_SDCARD_MISO_PIN
#define BOARD_SDCARD_SCLK_PIN CONFIG_SDCARD_SCLK_PIN
#ifdef SOC_SDMMC_HOST_SUPPORTED
#include "driver/sdmmc_host.h"
#endif
#endif

#define CONFIG_USE_SDSPI

// Mount path for the partition
const char *base_path = "/disk";
/* Function to initialize SPIFFS */
static esp_err_t init_fat(sdmmc_card_t **card_handle)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    esp_err_t ret = ESP_FAIL;
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
#ifdef CONFIG_USE_INTERNAL_FLASH
// Handle of the wear levelling library instance
    wl_handle_t wl_handle_1 = WL_INVALID_HANDLE;
    ESP_LOGI(TAG, "using internal flash");
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 9,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    ret = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &wl_handle_1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

#elif defined CONFIG_USE_EXTERNAL_SDCARD
    sdmmc_card_t *card;
    ESP_LOGI(TAG, "using external sdcard");
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = CONFIG_DISK_BLOCK_SIZE
    };

#ifdef CONFIG_USE_SDSPI
    ESP_LOGI(TAG, "Using SPI peripheral");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = 26;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(base_path, &host, &slot_config, &mount_config, &card);
#else
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = 15;
    slot_config.cmd = 7;
    slot_config.d0 = 4;
    // To use 1-line SD mode, change this to 1:
    slot_config.width = 1;
    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    //slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ret = esp_vfs_fat_sdmmc_mount(base_path, &host, &slot_config, &mount_config, &card);
#endif

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }

        return ret;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    if (card_handle) {
        *card_handle = card;
    }

#endif

    return ESP_OK;
}

static void button_ok_single_click_cb(void *arg)
{
    ESP_LOGI(TAG, "BTN OK: BUTTON_SINGLE_CLICK");
}

static void button_up_single_click_cb(void *arg)
{
    ESP_LOGI(TAG, "BTN UP: BUTTON_SINGLE_CLICK");
}

static void button_dw_single_click_cb(void *arg)
{
    ESP_LOGI(TAG, "BTN DW: BUTTON_SINGLE_CLICK");
}

static void button_menu_single_click_cb(void *arg)
{
    ESP_LOGI(TAG, "BTN MENU: BUTTON_SINGLE_CLICK");
}

void app_main(void)
{
    iot_board_init();
    iot_board_usb_set_mode(USB_DEVICE_MODE);
    iot_board_usb_device_set_power(true, true);
    /* Initialize file storage */

    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_OK_ID), BUTTON_SINGLE_CLICK, button_ok_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_UP_ID), BUTTON_SINGLE_CLICK, button_dw_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_DW_ID), BUTTON_SINGLE_CLICK, button_up_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_MN_ID), BUTTON_SINGLE_CLICK, button_menu_single_click_cb);

    ESP_ERROR_CHECK(init_fat(&mount_card));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    tinyusb_config_t tusb_cfg = {
        .descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false // In the most cases you need to use a `false` value
    };

    tinyusb_config_msc_t msc_cfg = {
        .pdrv = 0,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_ERROR_CHECK(tusb_msc_init(&msc_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
    bool led_state = false;
    while (1) { //TODO: monitor system if you want
        float v1 = iot_board_get_host_voltage();
        float v2 = iot_board_get_battery_voltage();
        ESP_LOGW(TAG, "Host voltage=%.2f, Battery voltage=%.2f", v1, v2);
        iot_board_led_all_set_state(led_state);
        led_state = !led_state;
        vTaskDelay(2000 / portTICK_RATE_MS);
    }

}
