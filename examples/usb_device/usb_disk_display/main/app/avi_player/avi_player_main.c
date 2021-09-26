// Copyright 2019-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "app.h"
#include "vidoplayer.h"
#include "bsp_esp32_s3_usb_otg_ev.h"


#define CONFIG_LCD_BUF_WIDTH 240
#define CONFIG_LCD_BUF_HIGHT 48

#define DEMO_SPI_MAX_TRANFER_SIZE (CONFIG_LCD_BUF_WIDTH * CONFIG_LCD_BUF_HIGHT * 2 + 64)
static TaskHandle_t task_hdl = NULL;
QueueHandle_t g_avi_player_queue_hdl = NULL;

void avi_player_task( void *pvParameters )
{
    hmi_event_t current_event;
    g_avi_player_queue_hdl = xQueueCreate( 5, sizeof(hmi_event_t));
    assert(g_avi_player_queue_hdl != NULL);
    uint8_t *lcd_buffer = (uint8_t *)heap_caps_malloc(DEMO_SPI_MAX_TRANFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(lcd_buffer != NULL);
    avi_play( BOARD_SDCARD_BASE_PATH "/badapple/badapple_color.avi", lcd_buffer, CONFIG_LCD_BUF_WIDTH, CONFIG_LCD_BUF_HIGHT, board_lcd_draw_image, 240, 240);
    while (1) {
        if(xQueueReceive(g_avi_player_queue_hdl, &current_event, portMAX_DELAY) != pdTRUE) continue;
        switch (current_event.id) {
            case BTN_CLICK_MENU:

                break;
            case BTN_CLICK_UP:

                break;
            case BTN_CLICK_DOWN:

                break;
            case BTN_CLICK_OK:

                break;
            default:

                break;
        }
    }
}

void avi_player_init(void)
{
    if (task_hdl == NULL)
    xTaskCreate(avi_player_task, "avi_player", 4096, NULL, TASK_APP_PRIO_MIN, &task_hdl);
}

void avi_player_deinit(void)
{

}