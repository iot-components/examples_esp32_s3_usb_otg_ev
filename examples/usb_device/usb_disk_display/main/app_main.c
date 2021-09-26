
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "hal/usb_hal.h"

#include "bsp_esp32_s3_usb_otg_ev.h"
#include "spi_bus.h"
#include "app.h"

static const char *TAG = "usb_msc_demo";
QueueHandle_t s_default_queue_hdl = NULL;

#define CONFIG_LCD_BUF_WIDTH 240
#define CONFIG_LCD_BUF_HIGHT 48

#define DEMO_SPI_MAX_TRANFER_SIZE (CONFIG_LCD_BUF_WIDTH * CONFIG_LCD_BUF_HIGHT * 2 + 64)

static void button_ok_single_click_cb(void *arg)
{
    hmi_event_t item_id = {.id = BTN_CLICK_OK};
    if (s_default_queue_hdl) {
        xQueueSend(s_default_queue_hdl, ( void *) &item_id, 0);
    }
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
    if (s_default_queue_hdl) {
        xQueueSend(s_default_queue_hdl, ( void *) &item_id, 0);
    }
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
    if (s_default_queue_hdl) {
        xQueueSend(s_default_queue_hdl, ( void *) &item_id, portMAX_DELAY);
    }
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
    if (s_default_queue_hdl) {
        xQueueSend(s_default_queue_hdl, ( void *) &item_id, portMAX_DELAY);
    }
    for (size_t i = 0; i < _app_driver_count; i++) {
        if( (_app_driver[i].p_queue_hdl) && *(_app_driver[i].p_queue_hdl) ) {
            xQueueSend(*(_app_driver[i].p_queue_hdl), ( void * ) &item_id, 0);
        }
    }
    ESP_LOGI(TAG, "BTN MENU: BUTTON_SINGLE_CLICK");
}

__NOINIT_ATTR int s_driver_index = 0;
static int s_active_driver_index = 0;
static bool app_kill(int index);

static bool app_launch(int index)
{
    assert(index >= 0 && index < _app_driver_count);
    if (_app_driver[index].flags.restart_before_init) {
        esp_restart();
    }
    _app_driver[index].init();
    s_active_driver_index = index;
    return true;
}

static bool app_kill(int index)
{
    assert(index >= 0 && index < _app_driver_count);
    if (_app_driver[index].deinit){
        _app_driver[index].deinit();
    }

    if (_app_driver[index].flags.restart_after_deinit || _app_driver[index].deinit == NULL) {
        s_driver_index = 0; //Prevention killed app restart
        esp_restart();
    }

    if(s_active_driver_index == index) //if killed current app
    s_active_driver_index = 0;
    return true;
}

static bool app_hide(int index)
{
    assert(index >= 0 && index < _app_driver_count);
    if (_app_driver[index].hide) {
        _app_driver[index].hide();
        return true;
    }
    return false;
}

static bool app_show(int index)
{
    assert(index >= 0 && index < _app_driver_count);
    if (_app_driver[index].show) {
        _app_driver[index].show();
        return true;
    }
    return false;
}

void app_manager_task( void *pvParameters)
{
    hmi_event_t current_event;
    const int driver_tail = _app_driver_count;
    const int driver_head = 1;
    s_default_queue_hdl = xQueueCreate(1, sizeof(hmi_event_t));
    assert(s_default_queue_hdl != NULL);
    while (1) {
        if(xQueueReceive(s_default_queue_hdl, &current_event, portMAX_DELAY) != pdTRUE) continue;
        switch (current_event.id) {
            case BTN_CLICK_MENU:
                if (s_active_driver_index != 0) {
                    app_kill(s_active_driver_index);
                    app_launch(0);
                }
                break;
            case BTN_CLICK_UP:
                if (++s_driver_index >= driver_tail)
                s_driver_index = driver_head;
                break;
            case BTN_CLICK_DOWN:
                if (--s_driver_index < driver_head)
                s_driver_index = driver_tail - 1;
                break;
            case BTN_CLICK_OK:
                if(s_active_driver_index == 0 && s_driver_index > 0 && s_driver_index < _app_driver_count) {
                    app_kill(0);
                    app_launch(s_driver_index);
                }
                break;
            default:
                ESP_LOGE(TAG, "not supported event %d", current_event.id);
                break;
        }
    }
}

void app_main(void)
{
    iot_board_init();
    iot_board_usb_set_mode(USB_DEVICE_MODE);
    iot_board_usb_device_set_power(false, false);
    usb_hal_context_t hal = {
        .use_external_phy = true
    };
    usb_hal_init(&hal);

    xTaskCreate(app_manager_task, "app_mng", 4096, NULL, TASK_APP_PRIO_MAX, NULL);

    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_OK_ID), BUTTON_SINGLE_CLICK, button_ok_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_UP_ID), BUTTON_SINGLE_CLICK, button_up_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_DW_ID), BUTTON_SINGLE_CLICK, button_dw_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_MN_ID), BUTTON_SINGLE_CLICK, button_menu_single_click_cb);

    if (esp_reset_reason() == ESP_RST_SW && s_driver_index > 0 && s_driver_index < _app_driver_count) {
        ESP_LOGI(TAG, "Restart to run APP: %s",  _app_driver[s_active_driver_index].app_name);
    } else {
        s_driver_index = 0;
    }
    _app_driver[s_driver_index].init();
    s_active_driver_index = s_driver_index;

    bool led_state = false;
    while(1) {
        iot_board_led_all_set_state(led_state);
        led_state = !led_state;
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
