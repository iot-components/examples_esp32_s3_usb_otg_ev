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
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"

#include "bsp_esp32_s3_usb_otg_ev.h"
#include "modem_board.h"
#include "modem_wifi.h"

#ifdef CONFIG_CDC_USE_TRACE_FACILITY
#include "esp_usbh_cdc.h"
#endif

static const char *TAG = "main";

#ifdef CONFIG_DUMP_SYSTEM_STATUS
#define TASK_MAX_COUNT 32
typedef struct {
    uint32_t ulRunTimeCounter;
    uint32_t xTaskNumber;
} taskData_t;

static taskData_t previousSnapshot[TASK_MAX_COUNT];
static int taskTopIndex = 0;
static uint32_t previousTotalRunTime = 0;

static taskData_t *getPreviousTaskData(uint32_t xTaskNumber)
{
    // Try to find the task in the list of tasks
    for (int i = 0; i < taskTopIndex; i++) {
        if (previousSnapshot[i].xTaskNumber == xTaskNumber) {
            return &previousSnapshot[i];
        }
    }
    // Allocate a new entry
    ESP_ERROR_CHECK(!(taskTopIndex < TASK_MAX_COUNT));
    taskData_t *result = &previousSnapshot[taskTopIndex];
    result->xTaskNumber = xTaskNumber;
    taskTopIndex++;
    return result;   
}

static void _system_dump()
{
    uint32_t totalRunTime;

    TaskStatus_t taskStats[TASK_MAX_COUNT];
    uint32_t taskCount = uxTaskGetSystemState(taskStats, TASK_MAX_COUNT, &totalRunTime);
    ESP_ERROR_CHECK(!(taskTopIndex < TASK_MAX_COUNT));
    uint32_t totalDelta = totalRunTime - previousTotalRunTime;
    float f = 100.0 / totalDelta;
    // Dumps the the CPU load and stack usage for all tasks
    // CPU usage is since last dump in % compared to total time spent in tasks. Note that time spent in interrupts will be included in measured time.
    // Stack usage is displayed as nr of unused bytes at peak stack usage.

    ESP_LOGI(TAG,"Task dump\n");
    ESP_LOGI(TAG,"Load\tStack left\tName\tPRI\n");

    for (uint32_t i = 0; i < taskCount; i++) {
        TaskStatus_t *stats = &taskStats[i];
        taskData_t *previousTaskData = getPreviousTaskData(stats->xTaskNumber);

        uint32_t taskRunTime = stats->ulRunTimeCounter;
        float load = f * (taskRunTime - previousTaskData->ulRunTimeCounter);
        ESP_LOGI(TAG,"%.2f \t%u \t%s \t%u\n", load, stats->usStackHighWaterMark, stats->pcTaskName, stats->uxBasePriority);

        previousTaskData->ulRunTimeCounter = taskRunTime;
    }
    ESP_LOGI(TAG, "Free heap=%d bigst=%d, internal=%d bigst=%d",
                        heap_caps_get_free_size(MALLOC_CAP_DEFAULT), heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
                        heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL), 
                        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    previousTotalRunTime = totalRunTime;
#ifdef CONFIG_CDC_USE_TRACE_FACILITY
    ESP_LOGI(TAG,"..............\n");
    usbh_cdc_print_buffer_msg();
#endif
}
#endif

void app_main(void)
{
    iot_board_init();
    iot_board_usb_set_mode(USB_HOST_MODE);
    iot_board_usb_device_set_power(true, true);
    /* Initialize NVS for Wi-Fi storage */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated and needs to be erased
         * Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Initialize default TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Waitting for modem powerup */
    vTaskDelay(pdMS_TO_TICKS(250));
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "     ESP 4G Cat.1 Wi-Fi Router");
    ESP_LOGI(TAG, "====================================");

    /* Initialize modem board. Dial-up internet */
    modem_config_t modem_config = MODEM_DEFAULT_CONFIG();
    esp_netif_t *ppp_netif = modem_board_init(&modem_config);
    assert(ppp_netif != NULL);

    /* Initialize Wi-Fi. Start Wi-Fi AP */
    modem_wifi_config_t modem_wifi_config = MODEM_WIFI_DEFAULT_CONFIG();
    modem_wifi_config.max_connection = CONFIG_EXAMPLE_MAX_STA_CONN;
    modem_wifi_config.channel = CONFIG_EXAMPLE_WIFI_CHANNEL;
    modem_wifi_config.password = CONFIG_EXAMPLE_WIFI_PASSWORD;
    modem_wifi_config.ssid = CONFIG_EXAMPLE_WIFI_SSID;
    esp_netif_t *ap_netif  = modem_wifi_init(WIFI_MODE_AP);
    assert(ap_netif != NULL);
    ESP_ERROR_CHECK(modem_wifi_set(&modem_wifi_config));
    ESP_ERROR_CHECK(modem_wifi_napt_enable());

    /* Dump system status */
#ifdef CONFIG_DUMP_SYSTEM_STATUS
    while (1) {
        _system_dump();
        vTaskDelay(3000);
    }
#endif
}
