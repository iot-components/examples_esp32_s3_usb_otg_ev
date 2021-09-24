
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
#include "esp_wifi.h"
#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_types.h"
#include "sdmmc_cmd.h"
#include "assert.h"
#include "bsp_esp32_s3_usb_otg_ev.h"
#include "display_printf.h"
#include "tusb_msc.h"
#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_types.h"
#include "qrcode.h"

static const char *TAG = "usb_msc_demo";

extern esp_err_t start_file_server(const char *base_path);

void display_card_info(const sdmmc_card_t* card)
{
    bool print_scr = false;
    bool print_csd = false;
    const char* type;
    char msg[64] = "";

    sprintf(msg, "Name: %s", card->cid.name);
    DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, msg);
    if (card->is_sdio) {
        type = "SDIO";
        print_scr = true;
        print_csd = true;
    } else if (card->is_mmc) {
        type = "MMC";
        print_csd = true;
    } else {
        type = (card->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC";
    }
    sprintf(msg, "Type: %s", type);
    DISPLAY_PRINTF_LINE("SD", 3, COLOR_RED, msg);
    if (card->max_freq_khz < 1000) {
        sprintf(msg, "Speed: %d kHz", card->max_freq_khz);
    } else {
        sprintf(msg, "Speed: %d MHz%s", card->max_freq_khz / 1000,
                card->is_ddr ? ", DDR" : "");
    }
    DISPLAY_PRINTF_LINE("SD", 4, COLOR_RED, msg);
    sprintf(msg, "Size: %lluMB", ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
    DISPLAY_PRINTF_LINE("SD", 5, COLOR_RED, msg);

    if (print_csd) {
        sprintf(msg, "CSD: ver=%d, sector_size=%d, capacity=%d read_bl_len=%d",
                card->csd.csd_ver,
                card->csd.sector_size, card->csd.capacity, card->csd.read_block_len);
    }
    if (print_scr) {
        sprintf(msg, "SCR: sd_spec=%d, bus_width=%d", card->scr.sd_spec, card->scr.bus_width);
    }

}

static esp_err_t init_flash_fat(const char* base_path)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    esp_err_t ret = ESP_FAIL;
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
    // Handle of the wear levelling library instance
    wl_handle_t wl_handle_1 = WL_INVALID_HANDLE;
    ESP_LOGI(TAG, "using internal flash");
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 9,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    ret = esp_vfs_fat_spiflash_mount(base_path, NULL, &mount_config, &wl_handle_1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    return ESP_OK;
}

#define QR_BUF_LEN_MAX ((((10) * 4 + 17) * ((10) * 4 + 17) + 7) / 8 + 1) // Calculates the number of bytes needed to store any Version 10 QR Code
static char s_wifi_qr_buffer[QR_BUF_LEN_MAX] = {0};
extern void start_dns_server(void);

void usb_wireless_disk_init(void)
{

    DISPLAY_PRINTF_INIT((scr_driver_t *)iot_board_get_handle(BOARD_LCD_ID));
    sdmmc_card_t *card_hdl = (sdmmc_card_t *)iot_board_get_handle(BOARD_SDCARD_ID);
    if (card_hdl == NULL) {
        DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, "SDCard Not Found!");
        DISPLAY_PRINTF_LINE("SD", 4, COLOR_PURPLE, "Using internal flash");
        init_flash_fat(BOARD_SDCARD_BASE_PATH);
    } else {
        display_card_info(card_hdl);
    }
    iot_board_usb_set_mode(USB_DEVICE_MODE);
    /* Initialize file storage */

    /* Start the file server */
#ifdef CONFIG_WIFI_HTTP_ACCESS
    ESP_ERROR_CHECK(iot_board_wifi_init());
    ESP_ERROR_CHECK(start_file_server(BOARD_SDCARD_BASE_PATH));
    //start_dns_server();
#endif

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

    DISPLAY_PRINTF_LINE("SD", 6, COLOR_BLUE, "Access files from");
    DISPLAY_PRINTF_LINE("SD", 7, COLOR_BLUE, "USB or Wi-Fi AP");
    vTaskDelay(500 / portTICK_RATE_MS);

    wifi_config_t wifi_cfg;
    esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);
    snprintf(s_wifi_qr_buffer, sizeof(s_wifi_qr_buffer), "WIFI:S:%s;T:%s;P:%s;H:%s;", wifi_cfg.ap.ssid, wifi_cfg.ap.password[0]?"WPA":"", wifi_cfg.ap.password, wifi_cfg.ap.ssid_hidden?"true":"false");
    esp_qrcode_config_t qr_cfg = PAINTER_QRCODE_CONFIG_DEFAULT();
    ESP_LOGI(TAG, "Scan below QR Code to Connect Wi-Fi\n");
    esp_qrcode_generate(&qr_cfg, s_wifi_qr_buffer);

    ESP_LOGI(TAG, "USB initialization DONE");
}
