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

typedef struct {
    char* app_name;
    char* icon_name;
    void (* init)(void);
    void (* deinit)(void);
} user_app_t;

static user_app_t const _app_driver[] =
{
    {
        .app_name = "USB Wireless Disk",
        .icon_name = "icon_01.jpg"
        .init = 

    },


}