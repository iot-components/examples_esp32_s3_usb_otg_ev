
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "bsp_esp32_s3_usb_otg_ev.h"
#include "spi_bus.h"
#include "app.h"

static const char *TAG = "usb_msc_demo";

#define CONFIG_LCD_BUF_WIDTH 240
#define CONFIG_LCD_BUF_HIGHT 48

#define DEMO_SPI_MAX_TRANFER_SIZE (CONFIG_LCD_BUF_WIDTH * CONFIG_LCD_BUF_HIGHT * 2 + 64)

static void button_ok_single_click_cb(void *arg)
{
    hmi_event_t item_id = {.id = BTN_CLICK_OK};
    for (size_t i = 0; i < _app_driver_count; i++) {
        if( (_app_driver[i].p_queue_hdl) && *(_app_driver[i].p_queue_hdl) ) {
            xQueueSend(*(_app_driver[i].p_queue_hdl), ( void * ) &item_id, 0);
        }
    }
    ESP_LOGI(TAG, "BTN OK: BUTTON_SINGLE_CLICK");
}

static void button_up_single_click_cb(void *arg)
{
    hmi_event_t item_id = {.id = BTN_CLICK_UP};
    for (size_t i = 0; i < _app_driver_count; i++) {
        if( (_app_driver[i].p_queue_hdl) && *(_app_driver[i].p_queue_hdl) ) {
            xQueueSend(*(_app_driver[i].p_queue_hdl), ( void * ) &item_id, 0);
        }
    }
    ESP_LOGI(TAG, "BTN UP: BUTTON_SINGLE_CLICK");
}

static void button_dw_single_click_cb(void *arg)
{
    hmi_event_t item_id = {.id = BTN_CLICK_DOWN};
    for (size_t i = 0; i < _app_driver_count; i++) {
        if( (_app_driver[i].p_queue_hdl) && *(_app_driver[i].p_queue_hdl) ) {
            xQueueSend(*(_app_driver[i].p_queue_hdl), ( void * ) &item_id, 0);
        }
    }
    ESP_LOGI(TAG, "BTN DW: BUTTON_SINGLE_CLICK");
}

static void button_menu_single_click_cb(void *arg)
{
    hmi_event_t item_id = {.id = BTN_CLICK_MENU};
    for (size_t i = 0; i < _app_driver_count; i++) {
        if( (_app_driver[i].p_queue_hdl) && *(_app_driver[i].p_queue_hdl) ) {
            xQueueSend(*(_app_driver[i].p_queue_hdl), ( void * ) &item_id, 0);
        }
    }
    ESP_LOGI(TAG, "BTN MENU: BUTTON_SINGLE_CLICK");
}

extern __NOINIT_ATTR int s_driver_index;

void app_main(void)
{
    iot_board_init();
    iot_board_usb_set_mode(USB_DEVICE_MODE);
    iot_board_usb_device_set_power(false, false);

    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_OK_ID), BUTTON_SINGLE_CLICK, button_ok_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_UP_ID), BUTTON_SINGLE_CLICK, button_up_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_DW_ID), BUTTON_SINGLE_CLICK, button_dw_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_MN_ID), BUTTON_SINGLE_CLICK, button_menu_single_click_cb);

    if (esp_reset_reason() == ESP_RST_SW && s_driver_index != 0) {
        /* code */
         _app_driver[s_driver_index].init();
    } else {
        s_driver_index = 0;
    }

    _app_driver[0].init();

    bool led_state = false;
    while (1) {
        iot_board_led_all_set_state(led_state);
        led_state = !led_state;
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
