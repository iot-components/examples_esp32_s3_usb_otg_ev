
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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
#include "display_printf.h"
#include "display_painter.h"
#include "tusb_msc.h"
#include "esp_app_format.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

static const char *TAG = "test_code";
QueueHandle_t event_queue_hdl = NULL;

typedef enum {
    TEST_FAIL = -1,
    TEST_SUCCESS,
    TEST_LCD,
    TEST_BUTTON,
    TEST_LED,
    TEST_USB_DEV
} test_process_t;

typedef enum {
    TEST_BTN_FAIL = -1,
    TEST_BTN_SUCCESS,
    TEST_BTN_OK,
    TEST_BTN_UP,
    TEST_BTN_DW,
    TEST_BTN_MN
} test_process_btn_t;

typedef enum {
    TEST_LCD_FAIL = -1,
    TEST_LCD_SUCCESS,
    TEST_LCD_RED,
    TEST_LCD_GREEN,
    TEST_LCD_BLUE
} test_process_lcd_t;

typedef enum {
    TEST_LED_FAIL = -1,
    TEST_LED_SUCCESS,
    TEST_LED_GREEN,
    TEST_LED_YELLOW
} test_process_led_t;

typedef enum {
    TEST_USB_DEV_FAIL = -1,
    TEST_USB_DEV_SUCCESS
} test_process_usb_dev_t;

typedef struct {
    test_process_t proc;
    union {
        test_process_btn_t proc_btn;
        test_process_lcd_t proc_lcd;
        test_process_usb_dev_t proc_usb_dev;
    };
} queue_event_t;

static void display_card_info(const sdmmc_card_t *card)
{
    bool print_scr = false;
    bool print_csd = false;
    const char *type;
    char msg[64] = "";

    sprintf(msg, "Name: %s", card->cid.name);
    DISPLAY_PRINTF_LINE(TAG, 2, COLOR_RED, msg);

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
    DISPLAY_PRINTF_LINE(TAG, 3, COLOR_RED, msg);

    if (card->max_freq_khz < 1000) {
        sprintf(msg, "Speed: %d kHz", card->max_freq_khz);
    } else {
        sprintf(msg, "Speed: %d MHz%s", card->max_freq_khz / 1000,
                card->is_ddr ? ", DDR" : "");
    }

    DISPLAY_PRINTF_LINE(TAG, 4, COLOR_RED, msg);
    sprintf(msg, "Size: %lluMB", ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_RED, msg);

    if (print_csd) {
        sprintf(msg, "CSD: ver=%d, sector_size=%d, capacity=%d read_bl_len=%d",
                card->csd.csd_ver,
                card->csd.sector_size, card->csd.capacity, card->csd.read_block_len);
    }

    if (print_scr) {
        sprintf(msg, "SCR: sd_spec=%d, bus_width=%d", card->scr.sd_spec, card->scr.bus_width);
    }

}

char target_name[32] = "0";
char flash_str[32] = "0";
char mac_addr_str[64] = "0";
char mac_addr_str_raw[32] = "0";
char test_file_name[255] = "0";
char result_file_name[255] = "0";
char test_folder[128] = "0";
char app_version[128] = "0";

void append_result(char *name , bool if_pass)
{
    char result[255] = "0";
    snprintf(result, sizeof(result), "%s, %s, \r\n", name, if_pass?"PASS":"Fail");
    FILE *fd = fopen(test_file_name, "a");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to write file : %s", test_file_name);
        return;
    }
    fwrite(result, 1,sizeof(result),fd);
    fclose(fd);
}

static esp_err_t init_flash_fat(const char *base_path)
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

static void button_ok_single_click_cb(void *arg)
{
    queue_event_t event = {
        .proc = TEST_BUTTON,
    };
    event.proc_btn = TEST_BTN_OK;
    xQueueOverwrite(event_queue_hdl, &event);
    ESP_LOGI(TAG, "BTN OK: BUTTON_SINGLE_CLICK");
}

static void button_up_single_click_cb(void *arg)
{
    queue_event_t event = {
        .proc = TEST_BUTTON,
    };
    event.proc_btn = TEST_BTN_UP;
    xQueueOverwrite(event_queue_hdl, &event);
    ESP_LOGI(TAG, "BTN UP: BUTTON_SINGLE_CLICK");
}

static void button_dw_single_click_cb(void *arg)
{
    queue_event_t event = {
        .proc = TEST_BUTTON,
    };
    event.proc_btn = TEST_BTN_DW;
    xQueueOverwrite(event_queue_hdl, &event);
    ESP_LOGI(TAG, "BTN DW: BUTTON_SINGLE_CLICK");
}

static void button_menu_single_click_cb(void *arg)
{
    queue_event_t event = {
        .proc = TEST_BUTTON,
    };
    event.proc_btn = TEST_BTN_MN;
    xQueueOverwrite(event_queue_hdl, &event);
    ESP_LOGI(TAG, "BTN MENU: BUTTON_SINGLE_CLICK");
}

static int _test_lcd()
{
    static test_process_lcd_t lcd_step = TEST_LCD_RED;
    int return_value = TEST_SUCCESS;
    queue_event_t current_event;
    DISPLAY_PRINTF_SET_FONT(Font24);
    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE(TAG, 1, COLOR_RED, "Test LCD");
    DISPLAY_PRINTF_LINE(TAG, 3, COLOR_GREEN, "Pass: OK        ");
    DISPLAY_PRINTF_LINE(TAG, 4, COLOR_RED, "Fail: MENU            ");
    xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

    while (lcd_step > 0) {
        switch (lcd_step) {
            case TEST_LCD_RED:
                painter_clear(COLOR_RED);
                painter_set_back_color(COLOR_RED);
                DISPLAY_PRINTF_LINE(TAG, 4, COLOR_WHITE, "  LCD RED    ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    lcd_step = TEST_LCD_GREEN;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Pass         ");
                    append_result("TEST_LCD_RED", true);
                } else if(current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_MN){
                    lcd_step = TEST_LCD_GREEN;
                    return_value = TEST_FAIL;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Fail        ");
                    append_result("TEST_LCD_RED", false);
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;

            case TEST_LCD_GREEN:
                painter_clear(COLOR_GREEN);
                painter_set_back_color(COLOR_GREEN);
                DISPLAY_PRINTF_LINE(TAG, 4, COLOR_WHITE, "  LCD GREEN  ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    lcd_step = TEST_LCD_BLUE;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Pass         ");
                    append_result("TEST_LCD_GREEN", true);
                } else if(current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_MN){
                    lcd_step = TEST_LCD_BLUE;
                    return_value = TEST_FAIL;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Fail        ");
                    append_result("TEST_LCD_GREEN", false);
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;

            case TEST_LCD_BLUE:
                painter_clear(COLOR_BLUE);
                painter_set_back_color(COLOR_BLUE);
                DISPLAY_PRINTF_LINE(TAG, 4, COLOR_WHITE, "  LCD BLUE   ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    lcd_step = TEST_LCD_SUCCESS;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Pass         ");
                    append_result("TEST_LCD_BLUE", true);
                } else if(current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_MN){
                    lcd_step = TEST_LCD_FAIL;
                    return_value = TEST_FAIL;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Fail        ");
                    append_result("TEST_LCD_BLUE", false);
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;
            default:
                break;
        }
    };

    painter_clear(COLOR_BLACK);
    painter_set_back_color(COLOR_BLACK);
    DISPLAY_PRINTF_LINE(TAG, 2, COLOR_RED, "  Test LCD");
    if (return_value == TEST_SUCCESS){
        DISPLAY_PRINTF_LINE(TAG, 4, COLOR_GREEN, "  All Pass    !");
    } else {
        DISPLAY_PRINTF_LINE(TAG, 4, COLOR_RED, "  Failed      !");
    }

    return return_value;
}

static int _test_button()
{
    static test_process_btn_t button_step = TEST_BTN_OK;
    queue_event_t current_event;
    int return_value = TEST_SUCCESS;
    DISPLAY_PRINTF_SET_FONT(Font24);
    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE(TAG, 1, COLOR_RED, "Test Button");
    const int threshold = 4;
    int fail_counter = 0;
    while (button_step > 0) {
        switch (button_step) {
            case TEST_BTN_OK:
                DISPLAY_PRINTF_LINE(TAG, 4, COLOR_BLUE, "  Press OK   ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == button_step) {
                    button_step = TEST_BTN_UP;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Pass        ");
                    append_result("TEST_BTN_OK", true);
                } else if(current_event.proc == TEST_BUTTON && current_event.proc_btn != button_step){
                    if(++fail_counter > threshold){
                        fail_counter = 0;
                        return_value = TEST_FAIL;
                        button_step = TEST_BTN_UP;
                        DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Fail        ");
                        append_result("TEST_BTN_OK", false);
                    } else {
                        DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, " Retry %d      ", threshold - fail_counter);
                    }
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;

            case TEST_BTN_UP:
                DISPLAY_PRINTF_LINE(TAG, 4, COLOR_BLUE, "  Press Up   ");
                DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "               ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == button_step) {
                    button_step = TEST_BTN_DW;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Pass        ");
                    append_result("TEST_BTN_UP", true);
                } else if(current_event.proc == TEST_BUTTON && current_event.proc_btn != button_step){
                    if(++fail_counter > threshold){
                        fail_counter = 0;
                        return_value = TEST_FAIL;
                        button_step = TEST_BTN_DW;
                        DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Fail        ");
                        append_result("TEST_BTN_UP", false);
                    } else {
                        DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, " Retry %d      ", threshold - fail_counter);
                    }
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;

            case TEST_BTN_DW:
                DISPLAY_PRINTF_LINE(TAG, 4, COLOR_BLUE, "  Press Down   ");
                DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "               ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == button_step) {
                    button_step = TEST_BTN_MN;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Pass        ");
                    append_result("TEST_BTN_DW", true);
                } else if(current_event.proc == TEST_BUTTON && current_event.proc_btn != button_step){
                    if(++fail_counter > threshold){
                        fail_counter = 0;
                        return_value = TEST_FAIL;
                        button_step = TEST_BTN_MN;
                        DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Fail        ");
                        append_result("TEST_BTN_DW", false);
                    } else {
                        DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, " Retry %d      ", threshold - fail_counter);
                    }
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;

            case TEST_BTN_MN:
                DISPLAY_PRINTF_LINE(TAG, 4, COLOR_BLUE, "  Press Menu   ");
                DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "               ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == button_step) {
                    button_step = TEST_BTN_SUCCESS;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Pass        ");
                    append_result("TEST_BTN_MN", true);
                } else if(current_event.proc == TEST_BUTTON && current_event.proc_btn != button_step){
                    if(++fail_counter > threshold){
                        fail_counter = 0;
                        return_value = TEST_FAIL;
                        button_step = TEST_BTN_FAIL;
                        DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "  Fail        ");
                        append_result("TEST_BTN_MN", false);
                    } else {
                        DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, " Retry %d      ", threshold - fail_counter);
                    }
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;

            default:
                break;
        }
    };

    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE(TAG, 2, COLOR_RED, "  Test Button");
    if (return_value == TEST_SUCCESS){
        DISPLAY_PRINTF_LINE(TAG, 4, COLOR_GREEN, "  All Pass    !");
    } else {
        DISPLAY_PRINTF_LINE(TAG, 4, COLOR_RED, "  Failed      !");
    }

    return return_value;
}

static int _test_led()
{
    static test_process_led_t led_step = TEST_LED_GREEN;
    queue_event_t current_event;
    int return_value = TEST_SUCCESS;
    DISPLAY_PRINTF_SET_FONT(Font24);
    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE(TAG, 1, COLOR_RED, "Test LED");
    DISPLAY_PRINTF_LINE(TAG, 3, COLOR_GREEN, "Pass: OK        ");
    DISPLAY_PRINTF_LINE(TAG, 4, COLOR_RED, "Fail: MENU            ");
    iot_board_led_all_set_state(false);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    DISPLAY_PRINTF_LINE(TAG, 3, COLOR_GREEN, "                 ");
    DISPLAY_PRINTF_LINE(TAG, 4, COLOR_RED, "                   ");

    while (led_step > 0) {
        switch (led_step) {
            case TEST_LED_GREEN:
                iot_board_led_set_state(BOARD_IO_LED_GREEN, true);
                DISPLAY_PRINTF_LINE(TAG, 3, COLOR_GREEN, "Green ON           ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);
                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    iot_board_led_all_set_state(false);
                    led_step = TEST_LED_YELLOW;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_GREEN, "  Pass         ");
                    append_result("TEST_LED_GREEN", true);
                } else if(current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_MN){
                    led_step = TEST_LED_YELLOW;
                    iot_board_led_all_set_state(false);
                    return_value = TEST_FAIL;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_RED, "  Fail        ");
                    append_result("TEST_LED_GREEN", false);
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;

            case TEST_LED_YELLOW:
                iot_board_led_set_state(BOARD_IO_LED_YELLOW, true);
                DISPLAY_PRINTF_LINE(TAG, 3, COLOR_YELLOW, "Yellow ON           ");
                DISPLAY_PRINTF_LINE(TAG, 5, COLOR_WHITE, "               ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    iot_board_led_all_set_state(false);
                    led_step = TEST_LED_SUCCESS;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_GREEN, "  Pass         ");
                    append_result("TEST_LED_YELLOW", true);
                } else if(current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_MN){
                    iot_board_led_all_set_state(false);
                    led_step = TEST_LED_FAIL;
                    return_value = TEST_FAIL;
                    DISPLAY_PRINTF_LINE(TAG, 5, COLOR_RED, "  Fail        ");
                    append_result("TEST_LED_YELLOW", false);
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;
            default:
                break;
        }
    };

    painter_clear(COLOR_BLACK);
    iot_board_led_all_set_state(false);
    DISPLAY_PRINTF_LINE(TAG, 2, COLOR_RED, "  Test LED");
    if (return_value == TEST_SUCCESS){
        DISPLAY_PRINTF_LINE(TAG, 4, COLOR_GREEN, "  All Pass    !");
    } else {
        DISPLAY_PRINTF_LINE(TAG, 4, COLOR_RED, "  Failed      !");
    }

    return return_value;
}

static void _cb_mount(int pdrv, void *arg)
{
    queue_event_t event = {
        .proc = TEST_USB_DEV,
    };

    event.proc_usb_dev = TEST_USB_DEV_SUCCESS;
    xQueueOverwrite(event_queue_hdl, &event);
    ESP_LOGI(TAG, "%s", __func__);
}

static void _cb_unmount(int pdrv, void *arg)
{
    ESP_LOGI(TAG, "%s", __func__);
}

static int _test_usb_dev()
{
    iot_board_usb_set_mode(USB_DEVICE_MODE);
    queue_event_t current_event;
    tinyusb_config_t tusb_cfg = {
        .descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false // In the most cases you need to use a `false` value
    };

    tinyusb_config_msc_t msc_cfg = {
        .pdrv = 1,
        .cb_mount = _cb_mount,
        .cb_unmount = _cb_unmount,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_ERROR_CHECK(tusb_msc_init(&msc_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE(TAG, 2, COLOR_RED, "Test USB Device");
    DISPLAY_PRINTF_LINE(TAG, 4, COLOR_GREEN, "Wait Insert..");

    while (1)
    {
        xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);
        if (current_event.proc != TEST_USB_DEV) continue;
        if (current_event.proc_usb_dev == TEST_USB_DEV_SUCCESS) {
            DISPLAY_PRINTF_LINE(TAG, 5, COLOR_GREEN, "   Pass !");
            return TEST_USB_DEV_SUCCESS;
        }
        DISPLAY_PRINTF_LINE(TAG, 5, COLOR_RED, "   Fail !");
        break;
    }
    
    return TEST_USB_DEV_FAIL;
}

void test_task(void *pvParameters)
{
    static test_process_t test_pro = TEST_LCD;
    int test_result_total = TEST_SUCCESS;

    for (size_t i = 0; i < 3; i++) {
        int test_result = 0;

        while (test_pro > 0) {
            switch (test_pro) {
                case TEST_LCD:
                    /* code */
                    ESP_LOGW(TAG, "Test LCD Start");
                    test_result = _test_lcd();
                    ESP_LOGI(TAG, "Test LCD Stop");
                    if (test_result == 0) {
                        test_pro = TEST_BUTTON;
                        append_result("TEST_LCD_ALL", true);
                    } else {
                        test_pro = TEST_BUTTON;
                        test_result_total = TEST_FAIL;
                        append_result("TEST_LCD_ALL", false);
                    }

                    break;

                case TEST_BUTTON:
                    ESP_LOGW(TAG, "Test Button Start");
                    test_result = _test_button();
                    ESP_LOGI(TAG, "Test Button Stop");

                    if (test_result == 0) {
                        test_pro = TEST_LED;
                        append_result("TEST_BUTTON_ALL", true);
                    } else {
                        test_pro = TEST_LED;
                        test_result_total = TEST_FAIL;
                        append_result("TEST_BUTTON_ALL", false);
                    }

                    break;

                case TEST_LED:
                    ESP_LOGW(TAG, "Test LED Start");
                    test_result = _test_led();
                    ESP_LOGI(TAG, "Test LED Stop");

                    if (test_result == 0) {
                        test_pro = TEST_USB_DEV;
                        append_result("TEST_LED_ALL", true);
                    } else {
                        test_pro = TEST_USB_DEV;
                        test_result_total = TEST_FAIL;
                        append_result("TEST_LED_ALL", false);
                    }

                    break;

                case TEST_USB_DEV:
                    ESP_LOGW(TAG, "Test USB Start");
                    test_result = _test_usb_dev();
                    ESP_LOGI(TAG, "Test USB Stop");

                    if (test_result == 0) {
                        test_pro = TEST_SUCCESS;
                        append_result("TEST_USB_DEV", true);
                    } else {
                        test_pro = TEST_FAIL;
                        test_result_total = TEST_FAIL;
                        append_result("TEST_USB_DEV", false);
                    }

                    break;

                default:
                    break;
            }

            vTaskDelay(1500 / portTICK_PERIOD_MS);
        };
    }

    uint32_t blink_delay = 0;
    int32_t restart_counter = 0;
    bool led_state = false;

    if (test_result_total == TEST_SUCCESS) {
        blink_delay = 1000;
        restart_counter = 6;
        painter_clear(COLOR_BLACK);
        DISPLAY_PRINTF_LINE(TAG, 4, COLOR_GREEN, "Test All Pass!");
        strncpy(result_file_name, test_file_name, sizeof(result_file_name));
        char *pos = strstr(result_file_name, "fail");
        memcpy(pos, "pass", 4);
        if (rename(test_file_name, result_file_name) != 0) {
            ESP_LOGW(TAG, "Try to delete old file");
            remove(result_file_name);
        }

        if (rename(test_file_name, result_file_name) != 0) {
            ESP_LOGE(TAG, "Rename failed!");
        }

    } else {
        painter_clear(COLOR_BLACK);
        blink_delay = 500;
        DISPLAY_PRINTF_LINE(TAG, 4, COLOR_GREEN, "Test Failed=%d!", test_pro);
    }

    while (1) { //TODO: monitor system if you want
        // float v1 = iot_board_get_host_voltage();
        // float v2 = iot_board_get_battery_voltage();
        // ESP_LOGW(TAG, "Host voltage=%.2f, Battery voltage=%.2f", v1, v2);
        iot_board_led_all_set_state(led_state);
        led_state = !led_state;
        vTaskDelay(blink_delay / portTICK_RATE_MS);
        if (--restart_counter > 0) DISPLAY_PRINTF_LINE(TAG, 5, COLOR_RED, "restart..%d", restart_counter);
        if (restart_counter == 0) esp_restart();
    }
}

static void _print_info()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("\n");
    ESP_LOGW(TAG, "-----------------MAC information-----------------");
    const esp_app_desc_t *app_desc = esp_ota_get_app_description();
    uint8_t derived_mac_addr[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(derived_mac_addr, ESP_MAC_WIFI_SOFTAP));
    snprintf(mac_addr_str_raw, sizeof(mac_addr_str_raw), "%02x_%02x_%02x_%02x_%02x_%02x",
             derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
             derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);
    snprintf(mac_addr_str, sizeof(mac_addr_str), "Mac, %02x:%02x:%02x:%02x:%02x:%02x,\r\n", 
             derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
             derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);
    ESP_LOGI(TAG, "SoftAP MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
             derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);

    ESP_LOGW(TAG, "------------------Chip information------------");
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG, "%dMB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    snprintf(target_name, sizeof(target_name), "Chip, %s r%01d,\r\n", CONFIG_IDF_TARGET, chip_info.revision);
    snprintf(flash_str, sizeof(flash_str), "Flash, %dMB %s,\r\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(TAG, "Minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());

    ESP_LOGW(TAG, "---------------Application information--------------");
    ESP_LOGI(TAG, "Project name:     %s", app_desc->project_name);
    ESP_LOGI(TAG, "App version:      %s", app_desc->version);
    sprintf(app_version, "Test Firmware, %s %s \r\n", app_desc->project_name, app_desc->version);
    printf("\n");
}

void app_main(void)
{
    _print_info();
    iot_board_init();
    iot_board_usb_device_set_power(false, false);

    event_queue_hdl = xQueueCreate(1, sizeof(queue_event_t));
    assert(event_queue_hdl != NULL);

    /* Initialize file storage */
    DISPLAY_PRINTF_INIT((scr_driver_t *)iot_board_get_handle(BOARD_LCD_ID));
    sdmmc_card_t *card_hdl = (sdmmc_card_t *)iot_board_get_handle(BOARD_SDCARD_ID);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_OK_ID), BUTTON_SINGLE_CLICK, button_ok_single_click_cb);

    DISPLAY_PRINTF_LINE(TAG, 7, COLOR_GREEN, "Press OK to Start");
    queue_event_t current_event;
    while (1) {
        xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);
        if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
            break;
        }
    }

    if (card_hdl == NULL) {
        DISPLAY_PRINTF_LINE(TAG, 2, COLOR_RED, "SDCard Not Found!");
        DISPLAY_PRINTF_LINE(TAG, 4, COLOR_PURPLE, "Using internal flash");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        display_card_info(card_hdl);
        ESP_ERROR_CHECK(init_flash_fat("/internal"));
        ESP_LOGW(TAG, "---------------Write information to SD Card--------------");
        snprintf(test_folder, sizeof(test_folder), "%s/test", BOARD_SDCARD_BASE_PATH);
        mkdir(test_folder, 0777);
        snprintf(test_file_name, sizeof(test_file_name), "%s/%s_fail.csv", test_folder, mac_addr_str_raw);
        FILE *fd = fopen(test_file_name, "w");
        if (!fd) {
            ESP_LOGE(TAG, "Failed to create file : %s", test_file_name);
            return;
        }
        fwrite(target_name, 1,sizeof(target_name),fd);
        fwrite(flash_str, 1,sizeof(flash_str),fd);
        fwrite(mac_addr_str, 1,sizeof(mac_addr_str),fd);
        fwrite(app_version, 1,sizeof(app_version),fd);
        fclose(fd);
        append_result("TEST_SDCARD", true);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_DW_ID), BUTTON_SINGLE_CLICK, button_dw_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_UP_ID), BUTTON_SINGLE_CLICK, button_up_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_MN_ID), BUTTON_SINGLE_CLICK, button_menu_single_click_cb);

    vTaskDelay(200 / portTICK_PERIOD_MS);

    xTaskCreate(test_task, "test", 4096, NULL, 2, NULL);

    while (1) { //TODO: monitor system if you want
        // float v1 = iot_board_get_host_voltage();
        // float v2 = iot_board_get_battery_voltage();
        // ESP_LOGW(TAG, "Host voltage=%.2f, Battery voltage=%.2f", v1, v2);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

}
