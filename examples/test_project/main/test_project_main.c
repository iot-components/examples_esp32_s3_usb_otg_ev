
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
    TEST_LED_ACTIVE,
    TEST_LED_INACTIVE
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
    queue_event_t current_event;
    DISPLAY_PRINTF_SET_FONT(Font24);
    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, "  Test LCD");
    DISPLAY_PRINTF_LINE("SD", 3, COLOR_WHITE, "Pass Press OK");
    xQueueReceive(event_queue_hdl, &current_event, 1000 / portTICK_PERIOD_MS);

    while (lcd_step > 0) {
        switch (lcd_step) {
            case TEST_LCD_RED:
                painter_clear(COLOR_RED);
                painter_set_back_color(COLOR_RED);
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_WHITE, "  LCD RED    ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    lcd_step = TEST_LCD_GREEN;
                }

                break;

            case TEST_LCD_GREEN:
                painter_clear(COLOR_GREEN);
                painter_set_back_color(COLOR_GREEN);
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_WHITE, "  LCD GREEN  ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    lcd_step = TEST_LCD_BLUE;
                }

                break;

            case TEST_LCD_BLUE:
                painter_clear(COLOR_BLUE);
                painter_set_back_color(COLOR_BLUE);
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_WHITE, "  LCD BLUE   ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    lcd_step = TEST_LCD_SUCCESS;
                }

                break;

            default:
                break;
        }
    };

    painter_clear(COLOR_BLACK);
    painter_set_back_color(COLOR_BLACK);
    DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, "  Test LCD");
    DISPLAY_PRINTF_LINE("SD", 4, COLOR_GREEN, "   Pass !");

    return lcd_step;
}

static int _test_button()
{
    static test_process_btn_t button_step = TEST_BTN_OK;
    queue_event_t current_event;
    DISPLAY_PRINTF_SET_FONT(Font24);
    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, "  Test Button");

    while (button_step > 0) {
        switch (button_step) {
            case TEST_BTN_OK:
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_BLUE, "  Press OK   ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == button_step) {
                    button_step = TEST_BTN_UP;
                }

                break;

            case TEST_BTN_UP:
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_BLUE, "  Press Up   ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == button_step) {
                    button_step = TEST_BTN_DW;
                }

                break;

            case TEST_BTN_DW:
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_BLUE, "  Press Down   ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == button_step) {
                    button_step = TEST_BTN_MN;
                }

                break;

            case TEST_BTN_MN:
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_BLUE, "  Press Menu   ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == button_step) {
                    button_step = TEST_BTN_SUCCESS;
                }

                break;

            default:
                break;
        }
    };

    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, "  Test Button");
    DISPLAY_PRINTF_LINE("SD", 4, COLOR_GREEN, "   Pass !");

    return button_step;
}

static int _test_led()
{
    static test_process_led_t led_step = TEST_LED_INACTIVE;
    static int led_loop = 2;
    queue_event_t current_event;
    DISPLAY_PRINTF_SET_FONT(Font24);
    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, "  Test LED");
    iot_board_led_all_set_state(true);

    while (led_step > 0) {
        switch (led_step) {
            case TEST_LED_INACTIVE:
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_BLUE, "  Press OK ");
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_GREEN, "  ON      ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    iot_board_led_all_set_state(false);
                    led_step = TEST_LED_ACTIVE;
                }

                break;

            case TEST_LED_ACTIVE:
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_BLUE, "  Press OK ");
                DISPLAY_PRINTF_LINE("SD", 4, COLOR_RED, "  OFF        ");
                xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);

                if (current_event.proc == TEST_BUTTON && current_event.proc_btn == TEST_BTN_OK) {
                    iot_board_led_all_set_state(true);
                    if (--led_loop > 0)
                    led_step = TEST_LED_INACTIVE;
                    else
                    led_step = TEST_LED_SUCCESS;
                }

                break;

            default:
                break;
        }
    };

    painter_clear(COLOR_BLACK);
    iot_board_led_all_set_state(false);
    DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, "  Test LED");
    DISPLAY_PRINTF_LINE("SD", 4, COLOR_GREEN, "   Pass !");

    return led_step;
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
        .pdrv = 0,
        .cb_mount = _cb_mount,
        .cb_unmount = _cb_unmount,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_ERROR_CHECK(tusb_msc_init(&msc_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
    painter_clear(COLOR_BLACK);
    DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, "Test USB Device");
    DISPLAY_PRINTF_LINE("SD", 4, COLOR_GREEN, "Wait Insert..");

    while (1)
    {
        xQueueReceive(event_queue_hdl, &current_event, portMAX_DELAY);
        if (current_event.proc != TEST_USB_DEV) continue;
        if (current_event.proc_usb_dev == TEST_USB_DEV_SUCCESS) {
            DISPLAY_PRINTF_LINE("SD", 5, COLOR_GREEN, "   Pass !");
            return TEST_USB_DEV_SUCCESS;
        }
        DISPLAY_PRINTF_LINE("SD", 5, COLOR_RED, "   Fail !");
        break;
    }
    
    return TEST_USB_DEV_FAIL;
}

void test_task(void *pvParameters)
{
    static test_process_t test_pro = TEST_LCD;

    for (size_t i = 0; i < 3; i++) {
        int test_result = 0;

        while (test_pro > 0) {
            switch (test_pro) {
                case TEST_LCD:
                    /* code */
                    test_result = _test_lcd();

                    if (test_result == 0) {
                        test_pro = TEST_BUTTON;
                    }

                    break;

                case TEST_BUTTON:
                    test_result = _test_button();

                    if (test_result == 0) {
                        test_pro = TEST_LED;
                    }

                    break;

                case TEST_LED:
                    test_result = _test_led();

                    if (test_result == 0) {
                        test_pro = TEST_USB_DEV;
                    }

                    break;

                case TEST_USB_DEV:
                    test_result = _test_usb_dev();

                    if (test_result == 0) {
                        test_pro = TEST_SUCCESS;
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

    if (test_pro != TEST_SUCCESS) {
        painter_clear(COLOR_BLACK);
        blink_delay = 500;
        DISPLAY_PRINTF_LINE("SD", 4, COLOR_GREEN, "Test Failed=%d!", test_pro);
    } else {
        blink_delay = 1000;
        restart_counter = 6;
        painter_clear(COLOR_BLACK);
        DISPLAY_PRINTF_LINE("SD", 4, COLOR_GREEN, "Test All Pass!");
    }

    while (1) { //TODO: monitor system if you want
        // float v1 = iot_board_get_host_voltage();
        // float v2 = iot_board_get_battery_voltage();
        // ESP_LOGW(TAG, "Host voltage=%.2f, Battery voltage=%.2f", v1, v2);
        iot_board_led_all_set_state(led_state);
        led_state = !led_state;
        vTaskDelay(blink_delay / portTICK_RATE_MS);
        if (--restart_counter > 0) DISPLAY_PRINTF_LINE("SD", 5, COLOR_RED, "restart..%d", restart_counter);
        if (restart_counter == 0) esp_restart();
    }
}

void app_main(void)
{
    iot_board_init();
    //iot_board_usb_device_set_power(true, true);

    /* Initialize file storage */
    DISPLAY_PRINTF_INIT((scr_driver_t *)iot_board_get_handle(BOARD_LCD_ID));
    sdmmc_card_t *card_hdl = (sdmmc_card_t *)iot_board_get_handle(BOARD_SDCARD_ID);

    if (card_hdl == NULL) {
        DISPLAY_PRINTF_LINE("SD", 2, COLOR_RED, "SDCard Not Found!");
        DISPLAY_PRINTF_LINE("SD", 4, COLOR_PURPLE, "Using internal flash");
        ESP_ERROR_CHECK(init_flash_fat(BOARD_SDCARD_BASE_PATH));
    } else {
        display_card_info(card_hdl);
    }

    DISPLAY_PRINTF_LINE("SD", 6, COLOR_BLUE, "Files Can be Access");
    DISPLAY_PRINTF_LINE("SD", 7, COLOR_BLUE, "From USB");

    event_queue_hdl = xQueueCreate(1, sizeof(queue_event_t));
    assert(event_queue_hdl != NULL);

    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_OK_ID), BUTTON_SINGLE_CLICK, button_ok_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_UP_ID), BUTTON_SINGLE_CLICK, button_dw_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_DW_ID), BUTTON_SINGLE_CLICK, button_up_single_click_cb);
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
