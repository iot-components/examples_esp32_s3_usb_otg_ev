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
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include "esp_pthread.h"
#include "esp_log.h"
#include "scan_codes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "app.h"
#include "hidapi.h"
#include "bsp_esp32_s3_usb_otg_ev.h"
#include "display_printf.h"
#include "screen_driver.h"

static const char *TAG = "hid_scanner";

#define USAGE          0x09
#define KEYBOARD     0x06
#define MOUSE         0x02

typedef enum {
    DEVICE_KEYBOARD,
    DEVICE_MOUSE,
    DEVICE_UNKNOWN,
} hid_device_t;

const char *button_state_str[] = {
    "Release",
    "Left button",
    "Right button",
    "Dummy",
    "Middle button",
};

typedef enum {
    BTN_RELEASE = 0,
    LEFT_BTN_PRESS = 1,
    RIGHT_BTN_PRESS = 2,
    MIDDLE_BTN_PRESS = 4,
} __attribute__((packed)) hid_button_state_t;

typedef struct {
    hid_button_state_t button_state;
    int8_t x_pos;
    int8_t y_pos;
    int8_t wheel;
} __attribute__((packed)) hid_mouse_event_t;

typedef struct {
    uint8_t mod_key;
    uint8_t reserved;
    uint8_t key[6];
} __attribute__((packed)) hid_keyboard_event_t;

_Static_assert(sizeof(hid_mouse_event_t) == 4, "Incorrect hid_mouse_event_t size");
_Static_assert(sizeof(hid_keyboard_event_t) == 8, "Incorrect hid_keyboard_event_t size");

static TaskHandle_t task_hdl = NULL;
QueueHandle_t g_keyboard_queue_hdl = NULL;

void print_keyboard_events(hid_device *device)
{

}

void mouse_task( void *pvParameters )
{
    hid_device *device = (hid_device *)pvParameters;
    while(1) {
        hid_mouse_event_t event;

        int res = hid_read(device, (uint8_t*)&event, sizeof(hid_mouse_event_t));
        if(res < 0) {
            printf("hid_read failed !!!\n");
            break;
        }

        if(event.button_state) {
            printf("%s\n", button_state_str[event.button_state]);
        }
        if(event.x_pos || event.y_pos) {    
            printf("x:%d y:%d\n", event.x_pos, event.y_pos);
        }
        if(event.wheel) {
            printf("Wheel %s\n", (event.wheel > 0) ? "Up" : "Down");
        }
    }
    hid_close(device);
    hid_exit();
}

hid_device_t get_device_type(hid_device *device)
{
    uint8_t report[128];
    memset(report, 0, sizeof(report));

    int size = hid_get_feature_report(device, report, 128);
    
    if(size == -1) {
        ESP_LOGE("HID", "Unable to read report\n");
        return DEVICE_UNKNOWN;
    }

    // ESP_LOG_BUFFER_HEX("", &report[1], size - 1);

    for(int i = 1; i < size; i++) {
        if(report[i] == USAGE) {
            if(report[i+1] == KEYBOARD) {
                return DEVICE_KEYBOARD;
            } else if(report[i+1] == MOUSE) {
                return DEVICE_MOUSE;
            }
        }
    }

    return DEVICE_UNKNOWN;
}

void keyboard_task( void *pvParameters )
{
    hid_device *device = (hid_device *)pvParameters;
    ESP_LOGI(TAG, "*********************************");
    scr_driver_t *p_lcd_driver = (scr_driver_t *)iot_board_get_handle(BOARD_LCD_ID);
    DISPLAY_PRINTF_INIT(p_lcd_driver);
    painter_clear(COLOR_BLACK);
    font_t font = display_printf_get_font();
    scr_info_t info;
    p_lcd_driver->get_info(&info);
    const uint32_t max_char_num = info.width / font.Width;
    const uint32_t max_line_num = info.height / font.Height;
    static int current_line_num = 0;
    static int current_char_num = -1;

    while(1) {
        hid_keyboard_event_t event;
        bool shift_pressed = false;

        int res = hid_read(device, (uint8_t*)&event, sizeof(hid_keyboard_event_t));
        if(res < 0) {
            printf("hid_read failed !!!\n");
            break;
        }

        if(event.mod_key == KEY_MOD_LSHIFT) {
            shift_pressed = true;
        }

        for(int i = 0; i < 6; i++) {
            uint8_t key = event.key[i];
            char current_char = '\0';
            if(key >= KEY_A && key <= KEY_Z) {
                current_char = shift_pressed ? key + 0x3D : key + 0x5D;
                printf("%c", current_char);
            } else if(key >= KEY_1 && key <= KEY_9) {
                current_char = key + 0x13;
                printf("%c", current_char);
            } else if(key == KEY_0) {
                current_char = '0';
                printf("0");
            } else if (key == KEY_ENTER) {
                current_char = '\n';
                printf("\n");
            } else if(key == KEY_SPACE) {
                current_char = ' ';
                printf(" ");
            } else if(key == KEY_BACKSPACE){
                current_char = ' ';
                painter_draw_char(current_char_num * font.Width, current_line_num * font.Height, current_char, &font, COLOR_WHITE);
                --current_char_num;
                if(current_char_num < 0) current_char_num = 0;
                continue;
            }

            if(current_char == '\0') continue;

            if (current_char == '\n' || ++current_char_num >= max_char_num) {
                current_line_num++;
                current_char_num = 0;
            }

            if (current_line_num >= max_line_num) {
                painter_clear(COLOR_BLACK);
                current_line_num = 0;
                current_char_num = 0;
            }
            
            if(current_char != '\n' && current_char != '\0') painter_draw_char(current_char_num * font.Width, current_line_num * font.Height, current_char, &font, COLOR_WHITE);
        }

        fflush(stdout);
    }
    hid_close(device);
    hid_exit();
    ESP_LOGI(TAG, "*********************************");
}

void keyboard_init(void)
{
    iot_board_usb_set_mode(USB_HOST_MODE);
    iot_board_usb_device_set_power(true, true);
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
    struct hid_device_info *devs, *cur_dev;
    
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&cfg);

    if (hid_init() != 0) return;

    devs = hid_enumerate(0x0, 0x0);
    cur_dev = devs;

    while (cur_dev) {
        printf("VID: %04hx PID: %04hx\n", cur_dev->vendor_id, cur_dev->product_id);
        printf("  Product:      %ls\n", cur_dev->product_string);
        printf("  Release:      %hx\n", cur_dev->release_number);
        printf("  Interface:    %d\n",  cur_dev->interface_number);
        printf("  Usage (page): 0x%hx (0x%hx)\n\n", cur_dev->usage, cur_dev->usage_page);
        vendor_id = cur_dev->vendor_id;
        product_id = cur_dev->product_id;
        cur_dev = cur_dev->next;
    }
    hid_free_enumeration(devs);

    // Open the device using the VID, PID, and optionally the Serial number.
    hid_device *handle = hid_open(vendor_id, product_id, NULL);
    if (!handle) {
        ESP_LOGE("HID", "Unable to open device\n");
         return;
    }

    hid_device_t device = get_device_type(handle);

    if (task_hdl == NULL){
        if (device == DEVICE_KEYBOARD) {
            ESP_LOGI(TAG, "Keyboard found\n");
            xTaskCreate(keyboard_task, "keyboard", 4096, (void *)handle, 2, &task_hdl);
        } else if (device == DEVICE_MOUSE) {
            ESP_LOGI(TAG, "Mouse found\n");
            xTaskCreate(mouse_task, "mouse", 4096, (void *)handle, 2, &task_hdl);
        } else {
            ESP_LOGE(TAG, "Unsupported deivce\n");
            hid_close(handle);
            hid_exit();
        }
    }

}

void keyboard_deinit(void)
{

}