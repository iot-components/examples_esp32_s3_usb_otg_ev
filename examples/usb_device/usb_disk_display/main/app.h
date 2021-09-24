// Copyright 2020-2021 Espressif Systems (Shanghai) PTE LTD
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

#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    char* app_name;
    char* icon_name;
    void (* init)(void);
    void (* deinit)(void);
    QueueHandle_t *p_queue_hdl;
    struct {
        uint16_t restart_before_init: 1;
        uint16_t restart_after_deinit: 1;
    } flags;
} user_app_t;

extern void usb_camera_init(void);
extern void usb_camera_deinit(void);
extern void avi_player_init(void);
extern void keyboard_init(void);
extern void app_menu_init(void);
extern void app_menu_deinit(void);
extern void usb_wireless_disk_init(void);
extern QueueHandle_t usb_camera_queue_hdl;
extern QueueHandle_t app_menu_queue_hdl;
extern QueueHandle_t avi_player_queue_hdl;
extern QueueHandle_t keyboard_queue_hdl;

typedef enum {
    BTN_CLICK_MENU = 0,
    BTN_CLICK_UP,
    BTN_CLICK_DOWN,
    BTN_CLICK_OK,
} hmi_event_id_t;

typedef struct {
    hmi_event_id_t id;
} hmi_event_t;

static user_app_t const _app_driver[] =
{
    {
        .app_name = "app menu",
        .icon_name = "esp_logo.jpg",
        .init = app_menu_init,
        .deinit = app_menu_deinit,
        .p_queue_hdl = &app_menu_queue_hdl,
    },
    {
        .app_name = "USB Camera",
        .icon_name = "icon_camera.jpg",
        .init = usb_camera_init,
        .deinit = usb_camera_deinit,
        .p_queue_hdl = &usb_camera_queue_hdl,
        .flags.restart_before_init = false,
        .flags.restart_after_deinit = false,
    },
    {
        .app_name = "USB Wireless Disk",
        .icon_name = "icon_file.jpg",
        .init = usb_wireless_disk_init,
        .deinit = NULL,
        .p_queue_hdl = NULL,
    },
    // {
    //     .app_name = "USB Wireless Disk",
    //     .icon_name = "icon_file.jpg",
    //     .init = usb_wireless_disk_init,
    //     .deinit = NULL,
    //     .p_queue_hdl = NULL,
    // },
    // {
    //     .app_name = "Keyboard Host",
    //     .icon_name = "icon_keyboard.jpg",
    //     .init = keyboard_init,
    //     .deinit = NULL,
    //     .p_queue_hdl = &keyboard_queue_hdl,
    // },
    // {
    //     .app_name = "AVI Player",
    //     .icon_name = "icon_video.jpg",
    //     .init = avi_player_init,
    //     .deinit = NULL,
    //     .p_queue_hdl = &avi_player_queue_hdl,
    // },
};

const static int _app_driver_count = sizeof(_app_driver) / sizeof(user_app_t);