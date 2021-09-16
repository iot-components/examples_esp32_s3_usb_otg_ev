// Copyright 2021 Espressif Systems (Shanghai) PTE LTD
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

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "soc/soc_memory_layout.h"
#include "soc/usb_types.h"
#include "usb/usb_host.h"
#include "private_include/hcd.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"

/* -----------------------------------------------------------------------------
------------------------------- Defines/Typedefs -------------------------------
----------------------------------------------------------------------------- */

// -------------------------- Configs and Constants ----------------------------

#define DEBOUNCE_TIME_MS                500
#define POST_RESET_DELAY_MS             100     //Some devices need time to spin up after a reset
#define ENUM_MPS                        64      //Use an MPS of 64 for the initial satges of enumeration
#define ENUM_CFG_DESC_SIZE_MAX          1023
#define ENUM_DATA_BUFF_MIN_SIZE         ENUM_CFG_DESC_SIZE_MAX     //Worst case enumeration is size of ctrl request
#define ENUM_CFG_IDX                    0       //Always get/set the first config descriptor during enumeration
#define PORT_NUM                        1

// ---------------------------- Typedefs and Enums -----------------------------

typedef enum {
    ENUM_STATE_GET_FIRST_DEVC_DESC,     //Fetching the device's device descriptor for the first time, with the MPS set to 64. Only the first 8 bytes can be considered valid.
    ENUM_STATE_SET_ADDR,
    ENUM_STATE_GET_SECOND_DEVC_DESC,    //Second device desc at correct MPS
    ENUM_STATE_GET_CONFIG_DESC,         //Get the first configuration descriptor
    ENUM_STATE_SET_CONFIG,              //Set the current config to number 1.
    ENUM_STATE_FINISH,
    ENUM_STATE_ENUMERATED
} enum_state_t;

typedef struct transfer_context         transfer_context_t;
typedef struct endpoint_obj             endpoint_t;
typedef struct interface_obj            intf_t;
typedef struct device_obj               devc_t;

// ---------------------------- Object Definitions ------------------------------

struct transfer_context {
    void (*callback)(urb_t *urb, void *arg);
    void *arg;
};

struct endpoint_obj {
    uint8_t address;
    hcd_pipe_handle_t pipe_hdl;
    usb_desc_ep_t *desc;
};

struct interface_obj {
    devc_t *device;    //Pointer to device
    usb_desc_intf_t *desc;
    int num_endpoints;
    endpoint_t *endpoints[];
};

struct device_obj {
    struct {
        bool opened;
        bool dflt_pipe_in_use;
        uint8_t devc_addr;
        uint8_t dflt_pipe_mps;
    } info;

    struct {
        uint8_t cfg_num;
        uint8_t num_intfs;
        uint8_t *desc_buff;
        intf_t **intf_list;
    } config;

    void *user_arg;
    usbh_devc_cb_t callback;
    usb_desc_device_t *desc;
    hcd_pipe_handle_t default_pipe;
    urb_t default_urb;
    SemaphoreHandle_t transfer_complete;
};

typedef struct {
    hcd_port_handle_t port_hdl;
    uint8_t num_conn_devices;
    QueueHandle_t event_queue;      // We use a queue for events that must be serialized
    usbh_driver_cb_t driver_cb;     // Driver callback and callback event
    devc_t *connected_device;       // Currently only support 1 device. Ideally this would be a list pointer
    enum_state_t enum_state;
} usbh_obj_t;

typedef struct {
    SemaphoreHandle_t transfer_complete;
    bool is_in_request;
    uint16_t *xfer_len;
    uint8_t *data;
} ctrl_transfer_context_t;

typedef struct {
    enum { PIPE_EVENT, PORT_EVENT } type;
    union
    {
        struct {
            hcd_port_handle_t handle;
            hcd_port_event_t event;
        } port;

        struct {
            hcd_pipe_handle_t handle;
            hcd_pipe_event_t event;
        } pipe;

    };
} usb_event_msg_t;

_Static_assert(sizeof(transfer_context_t) == sizeof(uint64_t),
               "Size of transfer_context_t is incorrect");

// ----------------------------- Driver Objects --------------------------------

static usbh_obj_t *p_usbh_obj = NULL;
static portMUX_TYPE usbh_lock = portMUX_INITIALIZER_UNLOCKED;

#define USBH_ENTER_CRITICAL_ISR()   portENTER_CRITICAL_ISR(&usbh_lock)
#define USBH_EXIT_CRITICAL_ISR()    portEXIT_CRITICAL_ISR(&usbh_lock)
#define USBH_ENTER_CRITICAL()       portENTER_CRITICAL(&usbh_lock)
#define USBH_EXIT_CRITICAL()        portEXIT_CRITICAL(&usbh_lock)

#define USBH_DEBUG

#ifdef USBH_DEBUG
#define PRINT_ESP_ERR(err) printf("ESP_ERR: %s, %s, %d\n", esp_err_to_name((err)), __FUNCTION__, __LINE__)
#else
#define PRINT_ESP_ERR(err)
#endif

#define RETURN_ON_ERROR(x) do {         \
    esp_err_t _err_ = (x);              \
    if (_err_ != ESP_OK) {              \
        PRINT_ESP_ERR(_err_);           \
        return _err_;                   \
    }                                   \
} while(0)

#define USBH_CHECK(cond, ret_val) ({            \
            if (!(cond)) {                      \
                PRINT_ESP_ERR(ret_val);         \
                return (ret_val);               \
            }                                   \
})
#define USBH_CHECK_FROM_CRIT(cond, ret_val) ({  \
            if (!(cond)) {                      \
                PRINT_ESP_ERR(ret_val);         \
                USBH_EXIT_CRITICAL();           \
                return ret_val;                 \
            }                                   \
})
#define USBH_ASSERT(cond)   ({if (!(cond)) {abort();}})

#define HCD_QUEUE_LEN       8

#define ENUM_TAG            "ENUM"

#define TRANSFER_DATA_MAX_BYTES 1024

/* -----------------------------------------------------------------------------
------------------------------- Helper Functions -------------------------------
----------------------------------------------------------------------------- */

static int round_up_to_mps_multiple(int xfer_size, int mps)
{
    int rem = xfer_size % mps;
    return (rem == 0) ? xfer_size : xfer_size + mps - rem;
}

static esp_err_t usbh_find_interface(usb_desc_intf_t **intf_desc, 
                                     const usb_desc_config_t *config_desc, 
                                     int bInterfaceNumber, 
                                     int bAlternateSetting)
{
    // Check arguments
    USBH_CHECK(config_desc != NULL, ESP_ERR_INVALID_ARG);
    USBH_CHECK(bInterfaceNumber < config_desc->bNumInterfaces, ESP_ERR_INVALID_ARG);

    esp_err_t intf_found = ESP_ERR_NOT_FOUND;
    uint8_t *intf_p = (uint8_t *)config_desc + USB_DESC_CONFIG_SIZE; // Set pointer right after configuration descriptor

    // Loop through all descriptors in current configuration and find interface descriptor with requested index
    do {
        usb_desc_intf_t *this_intf = (usb_desc_intf_t *)intf_p; // Cast pointer to Interface Descriptor (we check whether it REALLY is interface descriptor on next line)
        if ((this_intf->bDescriptorType == USB_W_VALUE_DT_INTERFACE) &&
            (this_intf->bInterfaceNumber == bInterfaceNumber) &&
            (this_intf->bAlternateSetting == bAlternateSetting || bAlternateSetting == -1) &&
            (this_intf->bNumEndpoints != 0)) {
            // Success: This is an interface descriptor with requested index
            intf_found = ESP_OK;
            *intf_desc = this_intf;
            break;
        } else {
            // Either this is NOT an interface descriptor or the interface descriptor has incorrect index: continue searching
            intf_p += this_intf->bLength; // Attention: pointer arithmetics!
        }
    } while(intf_p < ((uint8_t *)config_desc) + config_desc->wTotalLength); // Attention: pointer arithmetics!

    return intf_found;
}

static char *pipe_event_str[] = { "NONE", "URB_DONE", "INVALID", "XFER", "URB_NOT_AVAIL", "OVERFLOW", "STALL", };

static bool port_callback(hcd_port_handle_t port, hcd_port_event_t event, void *user_arg, bool in_isr)
{
    QueueHandle_t event_queue = (QueueHandle_t)user_arg;
    usb_event_msg_t msg = { .type = PORT_EVENT, .port = { .handle = port, .event = event } };

    BaseType_t xTaskWoken = pdFALSE;
    xQueueSendFromISR(event_queue, &msg, &xTaskWoken);
    return (xTaskWoken == pdTRUE);
}

static bool pipe_callback(hcd_pipe_handle_t pipe, hcd_pipe_event_t event, void *user_arg, bool in_isr)
{
    QueueHandle_t event_queue = (QueueHandle_t)user_arg;
    usb_event_msg_t msg = { .type = PIPE_EVENT, .pipe = { .handle = pipe, .event = event } };

    if (in_isr) {
        BaseType_t xTaskWoken = pdFALSE;
        xQueueSendFromISR(event_queue, &msg, &xTaskWoken);
        return (xTaskWoken == pdTRUE);
    } else {
        xQueueSend(event_queue, &msg, portMAX_DELAY);
        return false;
    }
}

static usb_speed_t port_speed;

static esp_err_t alloc_default_pipe_and_urb(usbh_obj_t *obj)
{
    hcd_port_handle_t port = obj->port_hdl;
    devc_t *device = obj->connected_device;

    hcd_port_command(port, HCD_PORT_CMD_RESET);

    RETURN_ON_ERROR( hcd_port_get_speed(port, &port_speed) );

    hcd_pipe_config_t config = {
        .callback = pipe_callback,
        .callback_arg = (void *)obj->event_queue,
        .context = NULL,
        .ep_desc = NULL,    //NULL EP descriptor to create a default pipe
        .dev_addr = 0,
        .dev_speed = port_speed,
    };

    assert(&device->default_pipe != NULL);
    RETURN_ON_ERROR( hcd_pipe_alloc(port, &config, &device->default_pipe) );

    size_t buffer_size = sizeof(usb_ctrl_req_t) + TRANSFER_DATA_MAX_BYTES;
    uint8_t *buffer = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
    if(buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    //Initialize URB and underlying transfer structure. Need to cast to dummy due to const fields
    usb_transfer_dummy_t *transfer_dummy = (usb_transfer_dummy_t *)&device->default_urb.transfer;
    transfer_dummy->data_buffer = buffer;
    transfer_dummy->num_isoc_packets = 0;
    return ESP_OK;
}

static esp_err_t urb_enqueue(hcd_pipe_handle_t pipe, urb_t *urb, int num_bytes)
{
    urb->transfer.num_bytes = num_bytes;
    return hcd_urb_enqueue(pipe, urb);
}

static esp_err_t enumerate_device(urb_t *urb)
{
    devc_t *device = p_usbh_obj->connected_device;
    enum_state_t state = p_usbh_obj->enum_state;
    urb_t *default_urb = &device->default_urb;
    uint8_t *p_data_buffer = (uint8_t *)device->default_urb.transfer.data_buffer;

    switch(state)
    {
        case ENUM_STATE_GET_FIRST_DEVC_DESC: {
            // RETURN_ON_ERROR( hcd_port_command(p_usbh_obj->port_hdl, HCD_PORT_CMD_RESET) ); // First Reset
            // vTaskDelay(pdMS_TO_TICKS(POST_RESET_DELAY_MS));
            USB_CTRL_REQ_INIT_GET_DEVICE_DESC((usb_ctrl_req_t *)p_data_buffer);
            RETURN_ON_ERROR( urb_enqueue(device->default_pipe, default_urb, ENUM_MPS) );
            ESP_LOGI(ENUM_TAG, "Get first device descriptor");
            break;
        }
        case ENUM_STATE_SET_ADDR: {
            USBH_ASSERT(urb->transfer.actual_num_bytes >= 8);
            usb_desc_device_t *devc_desc = (usb_desc_device_t *)(p_data_buffer + USB_CTRL_REQ_SIZE);
            device->info.dflt_pipe_mps = devc_desc->bMaxPacketSize0;
            // RETURN_ON_ERROR( hcd_port_command(p_usbh_obj->port_hdl, HCD_PORT_CMD_RESET) ); // Second Reset
            // vTaskDelay(pdMS_TO_TICKS(POST_RESET_DELAY_MS));

            int addr_to_set = ++(p_usbh_obj->num_conn_devices);
            device->info.devc_addr = addr_to_set;
            USB_CTRL_REQ_INIT_SET_ADDR((usb_ctrl_req_t *)p_data_buffer, addr_to_set);
            RETURN_ON_ERROR( urb_enqueue(device->default_pipe, default_urb, 0) );
            ESP_LOGI(ENUM_TAG, "Set Address");
            break;
        }
        case ENUM_STATE_GET_SECOND_DEVC_DESC: {
            int mps = device->info.dflt_pipe_mps;
            int dev_addr = device->info.devc_addr;
            RETURN_ON_ERROR( hcd_pipe_update_mps(device->default_pipe, mps) );
            RETURN_ON_ERROR( hcd_pipe_update_dev_addr(device->default_pipe, dev_addr) );

            int xfer_len = round_up_to_mps_multiple(USB_DESC_DEVICE_SIZE, mps);
            USB_CTRL_REQ_INIT_GET_DEVICE_DESC((usb_ctrl_req_t *)p_data_buffer);
            RETURN_ON_ERROR( urb_enqueue(device->default_pipe, default_urb, xfer_len) );
            ESP_LOGI(ENUM_TAG, "Get second device descriptor");
            break;
        }
        case ENUM_STATE_GET_CONFIG_DESC: {
            USBH_ASSERT(urb->transfer.actual_num_bytes == USB_DESC_DEVICE_SIZE);
            memcpy(device->desc, p_data_buffer + USB_CTRL_REQ_SIZE, USB_DESC_DEVICE_SIZE);

            int xfer_len = round_up_to_mps_multiple(ENUM_DATA_BUFF_MIN_SIZE, device->info.dflt_pipe_mps);
            USB_CTRL_REQ_INIT_GET_CONFIG_DESC((usb_ctrl_req_t *)p_data_buffer, ENUM_CFG_IDX, xfer_len);
            RETURN_ON_ERROR( urb_enqueue(device->default_pipe, default_urb, xfer_len) );
            ESP_LOGI(ENUM_TAG, "Get configuration descriptor");
            break;
        }
        case ENUM_STATE_SET_CONFIG: {
            int len_xferred = urb->transfer.actual_num_bytes;
            usb_desc_config_t *cfg_desc = (usb_desc_config_t *)(p_data_buffer + USB_CTRL_REQ_SIZE);
            USBH_ASSERT(len_xferred <= ENUM_DATA_BUFF_MIN_SIZE && len_xferred >= cfg_desc->wTotalLength);
            memcpy(device->config.desc_buff, cfg_desc, len_xferred);

            USB_CTRL_REQ_INIT_SET_CONFIG((usb_ctrl_req_t *)p_data_buffer, ENUM_CFG_IDX + 1);
            RETURN_ON_ERROR( urb_enqueue(device->default_pipe, default_urb, 0) );
            ESP_LOGI(ENUM_TAG, "Set configuration");
            break;
        }
        case ENUM_STATE_FINISH: {
            device->config.cfg_num = ENUM_CFG_IDX + 1;
            int bNumInterfaces = ((usb_desc_config_t *)device->config.desc_buff)->bNumInterfaces;
            intf_t **intf_list = calloc(bNumInterfaces, sizeof(intf_t *));
            USBH_CHECK(intf_list != NULL, ESP_ERR_NO_MEM);
            device->config.intf_list = intf_list;
            ESP_LOGI(ENUM_TAG, "Enumeration finished");
            p_usbh_obj->driver_cb(USBH_DRVR_EVT_NEW_DEVC);
            hcd_port_set_fifo_bias(p_usbh_obj->port_hdl, HCD_PORT_FIFO_BIAS_RX);
            break;
        }
        case ENUM_STATE_ENUMERATED: {
            ESP_LOGE(ENUM_TAG, "Already enumerated");
            break;
        }
    }

    p_usbh_obj->enum_state = (enum_state_t)(state + 1); // Trasition to next state
    return ESP_OK;
}

static bool is_enumeration_msg(hcd_pipe_handle_t pipe, usbh_obj_t *obj)
{
    return (pipe == obj->connected_device->default_pipe &&
            obj->enum_state != ENUM_STATE_ENUMERATED);
}

esp_err_t usbh_handle_events(uint32_t timeout_ms)
{
    USBH_CHECK(p_usbh_obj != NULL, ESP_ERR_INVALID_STATE);

    TickType_t timeout = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    usb_event_msg_t msg;
    while (xQueueReceive(p_usbh_obj->event_queue, &msg, timeout) == pdTRUE) {
        if(msg.type == PORT_EVENT) {
            hcd_port_event_t event = hcd_port_handle_event(msg.port.handle);
            if(event == HCD_PORT_EVENT_CONNECTION) {
                RETURN_ON_ERROR(alloc_default_pipe_and_urb(p_usbh_obj));
                enumerate_device(NULL);
            } else {
                ESP_LOGE("PORT", "Unhandled port event %d", msg.port.event);
            }
        } else {
            if(msg.pipe.event == HCD_PIPE_EVENT_URB_DONE) {
                urb_t *urb = hcd_urb_dequeue(msg.pipe.handle);
                if(is_enumeration_msg(msg.pipe.handle, p_usbh_obj)) {
                        enumerate_device(urb);
                } else {
                    transfer_context_t *ctx = urb->transfer.context;
                    ctx->callback(urb, ctx->arg);
                }
            } else {
                ESP_LOGE("PIPE", "Unhandled pipe event %s", pipe_event_str[msg.pipe.event]);
            }
        }
    }

    return ESP_OK;
}

static hcd_pipe_handle_t xfer_req_find_pipe(devc_t *device, uint8_t ep_addr)
{
    //Check if we are targetting the default endpoint
    if ((ep_addr & USB_B_ENDPOINT_ADDRESS_EP_NUM_MASK) == 0) {
        return device->default_pipe;
    }

    //Locate the correct endpoint
    for (int intf_idx = 0; intf_idx < device->config.num_intfs; intf_idx++) {
        intf_t *intf = device->config.intf_list[intf_idx];
        for (int ep_idx = 0; ep_idx < intf->num_endpoints; ep_idx++) {
            endpoint_t *ep = intf->endpoints[ep_idx];
            if (ep->address == ep_addr) {
                return ep->pipe_hdl;
            }
        }
    }

    return NULL;
}

// ----------------------------- Endpoint Object -------------------------------

static endpoint_t *endpoint_alloc(intf_t *intf_obj, usb_desc_ep_t *ep_desc, uint8_t devc_addr)
{
    endpoint_t *ep = malloc(sizeof(endpoint_t));
    if (ep == NULL) {
        return NULL;
    }

    hcd_pipe_config_t pipe_cfg = {
        .callback = pipe_callback,
        .callback_arg = p_usbh_obj->event_queue,
        .ep_desc = ep_desc,
        .dev_speed = port_speed,
        .dev_addr = devc_addr,
    };

    hcd_pipe_handle_t pipe_hdl = NULL;
    esp_err_t err = hcd_pipe_alloc(p_usbh_obj->port_hdl, &pipe_cfg, &pipe_hdl);
    if (err) {
        free(ep);
        return NULL;
    }

    ep->address = ep_desc->bEndpointAddress;
    ep->pipe_hdl = pipe_hdl;
    ep->desc = ep_desc;
    return ep;
}

static void endpoint_free(endpoint_t *ep)
{
    if (ep == NULL) {
        return;
    }
    hcd_pipe_free(ep->pipe_hdl);
    free(ep);
}

// ---------------------------- Interface Object -------------------------------

static void print_ep_desc(const usb_desc_ep_t * ep_desc);

static intf_t *intf_alloc(devc_t *devc, uint8_t bInterfaceNumber, int alt_interface)
{
    usb_desc_intf_t *intf_desc;
    const usb_desc_config_t *config_desc = (usb_desc_config_t *)devc->config.desc_buff;
    esp_err_t ret = usbh_find_interface(&intf_desc, config_desc, bInterfaceNumber, alt_interface);
    if (ret != ESP_OK) {
        return NULL;
    }
    const uint8_t num_endpoints = intf_desc->bNumEndpoints;
    USBH_ENTER_CRITICAL();
    int devc_addr = devc->info.devc_addr;
    USBH_EXIT_CRITICAL();
    // Allocate interface object
    intf_t *intf = calloc(1, sizeof(intf_t) + sizeof(endpoint_t *) * num_endpoints);
    if (intf == NULL) {
        return NULL;
    }

    // Find and allocate each endpoint object
    uint8_t *endpoint_p = (uint8_t *)intf_desc; // Start looking for Endpoint descriptors after Interface descriptor
    uint8_t endpoints_found = 0;
    for (int i = 0; i < num_endpoints; i++) {
        // Loop through all descriptors in current interface and find endpoint descriptors
        do {
            // Move to next descriptor
            endpoint_p += ((usb_desc_ep_t *)endpoint_p)->bLength;

            // Check whether we reached end of configuration descriptor
            if (endpoint_p >= ((uint8_t *)config_desc) + config_desc->wTotalLength) {
                goto err;
            }

            // Cast pointer to Endpoint Descriptor (we check whether it REALLY is endpoint descriptor on next line)
            usb_desc_ep_t *this_ep = (usb_desc_ep_t *)endpoint_p;

            if (this_ep->bDescriptorType == USB_W_VALUE_DT_ENDPOINT) {
                // Success: This is an endpoint descriptor: Allocate the endpoint
                intf->endpoints[i] = endpoint_alloc(intf, this_ep, devc_addr);
                if (intf->endpoints[i] == NULL) {
                    goto err;
                }
                endpoints_found++;
                break;
            } else if (this_ep->bDescriptorType == USB_W_VALUE_DT_INTERFACE) {
                // If we find another Interface descriptor, we must have missed our endpoint descriptors - Error in Device configuration
                goto err;
            }
        } while (endpoints_found != num_endpoints);
    }

    intf->device = devc;
    intf->desc = intf_desc;
    intf->num_endpoints = num_endpoints;
    return intf;

err:
    for (int i = 0; i < num_endpoints; i++) {
        endpoint_free(intf->endpoints[i]);
    }
    free(intf);
    return NULL;
}

static void intf_free(intf_t *intf)
{
    if (intf == NULL) {
        return;
    }
    for (int i = 0; i < intf->num_endpoints; i++) {
        endpoint_free(intf->endpoints[i]);
    }
    free(intf);
}

/* -----------------------------------------------------------------------------
------------------------------- Public Functions -------------------------------
----------------------------------------------------------------------------- */

// ------------------------------ Driver State ---------------------------------

static void usbh_cleanup(usbh_obj_t *obj)
{
    if(!obj) {
        return;
    }

    devc_t *device = p_usbh_obj->connected_device;

    if(!device) {
        free(obj);
        return;
    }

    for(int i = 0; i < device->config.num_intfs; i++) {
        intf_free(device->config.intf_list[i]);
    }

    if(obj->event_queue) {
        vQueueDelete(obj->event_queue);
    }

    if(device->transfer_complete) {
        vSemaphoreDelete(device->transfer_complete);
    }

    if(device->default_pipe) {
        hcd_pipe_free(device->default_pipe);
    }

    if(device->default_urb.transfer.data_buffer) {
        free(device->default_urb.transfer.data_buffer);
    }

    if(obj->port_hdl) {
        hcd_port_deinit(obj->port_hdl);
        hcd_uninstall();
    }

    free(device->config.desc_buff);
    free(device->desc);
    free(device);
    free(obj);
}

esp_err_t usbh_install(usbh_config_t *config)
{
    devc_t *device = NULL;
    esp_err_t ret = ESP_ERR_NO_MEM;

    USBH_ENTER_CRITICAL();
    USBH_CHECK_FROM_CRIT(p_usbh_obj == NULL, ESP_ERR_INVALID_STATE);
    USBH_EXIT_CRITICAL();


    //Allocate driver object
    usbh_obj_t *obj = calloc(1, sizeof(usbh_obj_t));
    if (obj == NULL) {
        goto err;
    }

    device = calloc(1, sizeof(devc_t));
    if(device == NULL) {
        goto err;
    }

    device->desc = calloc(1, sizeof(usb_desc_device_t));
    if(device->desc == NULL) {
        goto err;
    }

    device->config.desc_buff = calloc(1, ENUM_CFG_DESC_SIZE_MAX);
    if(device->config.desc_buff == NULL) {
        goto err;
    }

    obj->event_queue = xQueueCreate(HCD_QUEUE_LEN, sizeof(usb_event_msg_t));
    device->transfer_complete = xSemaphoreCreateBinary();
    if (!obj->event_queue || !device->transfer_complete) {
        goto err;
    }
    obj->driver_cb = config->driver_cb;

    hcd_config_t hcd_config = { .intr_flags = ESP_INTR_FLAG_LEVEL1 };

    ret = hcd_install(&hcd_config);
    if (ret != ESP_OK) {
        goto err;
    }

    hcd_port_config_t port_config = {
        .callback = port_callback,
        .callback_arg = obj->event_queue,
        .context = NULL,
    };

    ret = hcd_port_init(PORT_NUM, &port_config, &obj->port_hdl);
    if(ret != ESP_OK) {
        goto err;
    }

    obj->connected_device = device;

    //Initialize usbh object
    USBH_ENTER_CRITICAL();
    p_usbh_obj = obj;
    USBH_EXIT_CRITICAL();
    return ESP_OK;

err:
    usbh_cleanup(obj);
    return ret;
}

esp_err_t usbh_uninstall()
{
    USBH_CHECK_FROM_CRIT(p_usbh_obj != NULL, ESP_ERR_INVALID_STATE);
    usbh_cleanup(p_usbh_obj);
    return ESP_OK;
}

esp_err_t usbh_start()
{
    USBH_ENTER_CRITICAL();
    USBH_CHECK_FROM_CRIT(p_usbh_obj != NULL, ESP_ERR_INVALID_STATE);
    USBH_EXIT_CRITICAL();

    return hcd_port_command(p_usbh_obj->port_hdl, HCD_PORT_CMD_POWER_ON);
}

esp_err_t usbh_stop()
{
    return ESP_OK;
}


// ----------------------------- Device Handling -------------------------------

esp_err_t usbh_devc_get_list(usbh_devc_t ***devc_list, int *num_conn_devices)
{
    USBH_ENTER_CRITICAL();
    USBH_CHECK_FROM_CRIT(p_usbh_obj != NULL, ESP_ERR_INVALID_STATE);
    //We only support one connected device
    USBH_CHECK_FROM_CRIT(p_usbh_obj->num_conn_devices == 1, ESP_ERR_NOT_FOUND);
    USBH_EXIT_CRITICAL();

    //Create device objects and list of one pointer to device objects
    usbh_devc_t **list = malloc(sizeof(usbh_devc_t *));
    usbh_devc_t *devc = malloc(sizeof(usbh_devc_t));
    if (list == NULL || devc == NULL) {
        goto err;
    }

    USBH_ENTER_CRITICAL();
    devc->dev_addr = p_usbh_obj->connected_device->info.devc_addr;
    devc->dev_desc = p_usbh_obj->connected_device->desc;
    USBH_EXIT_CRITICAL();
    list[0] = devc;
    *devc_list = list;
    *num_conn_devices = 1;
    return ESP_OK;

err:
    free(devc);
    free(list);
    return ESP_ERR_NO_MEM;
}

esp_err_t usbh_devc_free_list(usbh_devc_t **devc_list)
{
    USBH_CHECK(devc_list != NULL, ESP_ERR_INVALID_ARG);
    free(devc_list[0]);
    free(devc_list);
    return ESP_OK;
}

esp_err_t usbh_devc_open(usbh_devc_t *devc, usbh_devc_cb_t callback, void *user_arg, usbh_devc_hdl_t *devc_hdl)
{
    //Check arguments
    USBH_CHECK(devc != NULL && callback != NULL && devc_hdl != NULL, ESP_ERR_INVALID_ARG);
    USBH_ENTER_CRITICAL();
    //Check state
    USBH_CHECK_FROM_CRIT(p_usbh_obj != NULL && p_usbh_obj->connected_device != NULL
                         && (!p_usbh_obj->connected_device->info.opened), ESP_ERR_INVALID_STATE);
    //Check device
    devc_t *device = p_usbh_obj->connected_device;
    USBH_ASSERT(device != NULL);
    USBH_ASSERT(device->desc == devc->dev_desc);
    USBH_ASSERT(device->info.devc_addr == devc->dev_addr);
    //Update device lists and device object itself
    device->info.opened = 1;
    device->user_arg = user_arg;
    device->callback = callback;
    //Add device handle to the internal xfer object used for control transfers
    // device->dflt_pipe.xfer_req_intrnl->xfer_req_obj->devc_hdl = (usbh_devc_hdl_t *)device;
    USBH_EXIT_CRITICAL();

    *devc_hdl = (usbh_devc_hdl_t *)device;
    return ESP_OK;
}

esp_err_t usbh_devc_close(usbh_devc_hdl_t devc_hdl)
{
    devc_t *device = (devc_t *)devc_hdl;
    device->info.opened = 0;
    return ESP_OK;
}

esp_err_t usbh_get_devc_status(usbh_devc_hdl_t devc_hdl, usbh_devc_status_t *devc_info)
{
    return ESP_OK;
}

esp_err_t usbh_reset_device(usbh_devc_hdl_t devc_hdl)
{
    return ESP_OK;
}

// --------------------------- Interface Handling ------------------------------

esp_err_t usbh_intf_claim(usbh_devc_hdl_t devc_hdl, int interface_number, int alt_interface, usbh_intf_hdl_t *intf_hdl)
{
    // Check arguments
    USBH_CHECK(devc_hdl != NULL && intf_hdl != NULL, ESP_ERR_INVALID_ARG);
    devc_t *devc = (devc_t *)devc_hdl;

    // Check state
    USBH_ENTER_CRITICAL();
    USBH_CHECK_FROM_CRIT(devc->info.opened, ESP_ERR_INVALID_STATE);
    uint8_t *cfg_desc_buff = devc->config.desc_buff;
    int bNumInterfaces = ((usb_desc_config_t *)cfg_desc_buff)->bNumInterfaces;
    USBH_ASSERT(bNumInterfaces > 0);
    USBH_EXIT_CRITICAL();

    // Allocate interface
    intf_t *intf = intf_alloc(devc, interface_number, alt_interface);
    if (intf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Add interface to list of open interfaces
    USBH_ENTER_CRITICAL();
    devc->config.intf_list[devc->config.num_intfs] = intf;
    devc->config.num_intfs++;
    USBH_EXIT_CRITICAL();
    *intf_hdl = (usbh_intf_hdl_t) intf;
    return ESP_OK;
}

esp_err_t usbh_intf_release(usbh_devc_hdl_t devc_hdl, usbh_intf_hdl_t intf_hdl)
{
    devc_t *device = (devc_t *)devc_hdl;
    intf_t *intf = (intf_t *)intf_hdl;
    intf_t **intf_list = device->config.intf_list;

    for(int i = 0; i < device->config.num_intfs; i++) {
        if(intf_list[i] == intf) {
            for(int ep = 0; ep < intf->num_endpoints; ep++) {
                endpoint_free(intf->endpoints[ep]);
            }
            free(intf);
            intf_list[i] = NULL;
            device->config.num_intfs--;
            memcpy(&intf_list[i], &intf_list[i+1], sizeof(intf_t *)*(device->config.num_intfs - i));
        }
    }
    return ESP_OK;
}

esp_err_t usbh_intf_flush_pipe(usbh_intf_hdl_t intf_hdl, uint8_t bEndpointAddress)
{
    return ESP_OK;
}

esp_err_t usbh_intf_get_status(usbh_intf_hdl_t intf_hdl, usbh_intf_status_t *intf_status)
{
    return ESP_OK;
}

// -------------------------------- Async I/O ----------------------------------

static void async_transfer_cb(urb_t *urb, void *arg)
{
    usbh_xfer_req_t *request = (usbh_xfer_req_t *)arg;

    // Invoke user's callback
    request->callback(urb, request->user_arg);
}

esp_err_t usbh_xfer_req_submit(usbh_xfer_req_t *xfer_req)
{
    USBH_CHECK(xfer_req != NULL, ESP_ERR_INVALID_ARG);
    USBH_CHECK(esp_ptr_dma_capable(xfer_req->urb.transfer.data_buffer), ESP_ERR_INVALID_ARG);

    hcd_pipe_handle_t pipe = xfer_req_find_pipe((devc_t*)xfer_req->devc_hdl, xfer_req->bEndpointAddress);
    USBH_CHECK(pipe != NULL, ESP_ERR_NOT_FOUND);

    transfer_context_t *context = (transfer_context_t *)&xfer_req->reserved;

    context->callback = async_transfer_cb;
    context->arg = (void *)xfer_req;
    xfer_req->urb.transfer.context = context;

    return hcd_urb_enqueue(pipe, &xfer_req->urb);
}

esp_err_t usbh_xfer_req_cancel(usbh_xfer_req_t *xfer_req)
{
    return ESP_OK;
}


// -------------------------------- Sync I/O -----------------------------------

#define ENDPOINT_DIR_MASK		0x80

static inline bool is_out_request(usb_ctrl_req_t *req)
{
    return (req->bRequestType & ENDPOINT_DIR_MASK) == 0;
}

static inline bool is_in_request(usb_ctrl_req_t *req)
{
    return (req->bRequestType & ENDPOINT_DIR_MASK) != 0;
}

static esp_err_t wait_for_transfer_completion(uint32_t timeout_ms)
{
    timeout_ms = timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;
    int taken = xSemaphoreTake(p_usbh_obj->connected_device->transfer_complete, timeout_ms);
    return taken ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void ctrl_transfer_cb(urb_t *urb, void *arg)
{
    ctrl_transfer_context_t *ctx = (ctrl_transfer_context_t *)arg ;

    if (ctx->is_in_request) {
        memcpy(ctx->data, urb->transfer.data_buffer + USB_CTRL_REQ_SIZE, urb->transfer.actual_num_bytes);
    }

    *(ctx->xfer_len) = urb->transfer.actual_num_bytes;

    xSemaphoreGive(ctx->transfer_complete);
}

esp_err_t usbh_ctrl_xfer(usbh_devc_hdl_t devc_hdl, usb_ctrl_req_t *req, uint8_t *data, uint16_t *xfer_len, uint32_t timeout_ms)
{
    uint8_t *buffer = heap_caps_calloc(1, USB_CTRL_REQ_SIZE + 64, MALLOC_CAP_DMA);
    if(buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    devc_t *device = (devc_t *)devc_hdl;
    urb_t *urb = &device->default_urb;
    *xfer_len = 0;

    memcpy(buffer, req, USB_CTRL_REQ_SIZE);
    if (is_out_request(req)) {
        memcpy(buffer + USB_CTRL_REQ_SIZE, data, req->wLength);
    }

    ctrl_transfer_context_t ctrl_context = {
        .transfer_complete = device->transfer_complete,
        .is_in_request = is_in_request(req),
        .xfer_len = xfer_len,
        .data = data,
    };

    transfer_context_t transfer_context =  {
        .callback = ctrl_transfer_cb,
        .arg = &ctrl_context,
    };
    usb_transfer_dummy_t *transfer_dummy = (usb_transfer_dummy_t *)&urb->transfer;
    transfer_dummy->data_buffer = buffer;
    transfer_dummy->num_bytes = req->wLength;
    transfer_dummy->timeout = timeout_ms;
    transfer_dummy->context = &transfer_context;

    esp_err_t err = hcd_urb_enqueue(device->default_pipe, urb);
    if(err) {
        free(buffer);
        return err;
    }

    err = wait_for_transfer_completion(timeout_ms);
    free(buffer);
    return err;
}

esp_err_t usbh_bulk_xfer(usbh_devc_hdl_t devc_hdl, uint8_t bEndpointAddress, uint8_t *data, int xfer_len, uint32_t timeout_ms)
{
    return ESP_OK;
}

esp_err_t usbh_intr_xfer(usbh_devc_hdl_t devc_hdl, uint8_t bEndpointAddress, uint8_t *data, int xfer_len, uint32_t timeout_ms)
{
    return ESP_OK;
}

// --------------------------- Chapter 9 Functions -----------------------------

esp_err_t usbh_desc_get_devc(usbh_devc_hdl_t devc_hdl, const usb_desc_device_t **devc_desc)
{
    USBH_CHECK(devc_hdl != NULL && devc_desc != NULL, ESP_ERR_INVALID_ARG);

    USBH_ENTER_CRITICAL();
    devc_t *device = (devc_t *)devc_hdl;
    *devc_desc = device->desc;
    USBH_EXIT_CRITICAL();
    return ESP_OK;
}

esp_err_t usbh_desc_get_active_config(usbh_devc_hdl_t devc_hdl, const usb_desc_config_t **cfg_desc)
{
    return usbh_desc_get_full_config(devc_hdl, 0, (uint8_t **)cfg_desc);
}

// @todo Support more than one config descriptor
esp_err_t usbh_desc_get_full_config(usbh_devc_hdl_t devc_hdl, uint8_t cfg_idx, uint8_t **cfg_desc_buff)
{
    devc_t *device = (devc_t *)devc_hdl;

    USBH_CHECK(devc_hdl != NULL && cfg_desc_buff != NULL, ESP_ERR_INVALID_ARG);
    USBH_CHECK(cfg_idx <= device->desc->bNumConfigurations - 1, ESP_ERR_INVALID_ARG);

    *cfg_desc_buff = device->config.desc_buff;

    return ESP_OK;
}


esp_err_t usbh_desc_get_intf(usbh_intf_hdl_t intf_hdl, const usb_desc_intf_t **intf_desc)
{
    USBH_CHECK(intf_hdl != NULL && intf_desc != NULL, ESP_ERR_INVALID_ARG);
    intf_t *intf_obj = (intf_t *)intf_hdl;

    *intf_desc = intf_obj->desc;
    return ESP_OK;
}

esp_err_t usbh_desc_get_ep(usbh_intf_hdl_t intf_hdl, int ep_idx, const usb_desc_ep_t **ep_desc)
{
    USBH_CHECK(intf_hdl != NULL && ep_desc != NULL, ESP_ERR_INVALID_ARG);
    intf_t *intf_obj = (intf_t *)intf_hdl;
    USBH_CHECK(ep_idx < intf_obj->num_endpoints, ESP_ERR_NOT_FOUND);

    *ep_desc = intf_obj->endpoints[ep_idx]->desc;
    return ESP_OK;
}

static void print_ep_desc(const usb_desc_ep_t * ep_desc)
{
    //Print contents of endpoint descriptor
    printf("\t\tbLength %d\n", ep_desc->bLength);
    printf("\t\tbDescriptorType %d\n", ep_desc->bDescriptorType);
    printf("\t\tbEndpointAddress 0x%x\tEP %d %s\n", ep_desc->bEndpointAddress,
                                                    (ep_desc->bEndpointAddress & 0xF),
                                                    (ep_desc->bEndpointAddress & 0x80) ? "IN" : "OUT");
    const char *ep_type_str;
    int type = ep_desc->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK;
    switch (type) {
        case USB_BM_ATTRIBUTES_XFER_CONTROL:
            ep_type_str = "CTRL";
            break;
        case USB_BM_ATTRIBUTES_XFER_ISOC:
            ep_type_str = "ISOC";
            break;
        case USB_BM_ATTRIBUTES_XFER_BULK:
            ep_type_str = "BULK";
            break;
        case USB_BM_ATTRIBUTES_XFER_INT:
            ep_type_str = "INT";
            break;
        default:
            ep_type_str = NULL;
            break;
    }
    printf("\t\tbmAttributes 0x%x\t%s\n", ep_desc->bmAttributes, ep_type_str);
    printf("\t\twMaxPacketSize %d\n", ep_desc->wMaxPacketSize);
    printf("\t\tbInterval %d\n", ep_desc->bInterval);
    printf("\t\t++++++++++++++++\n");
}

esp_err_t usbh_print_intf_desc(usbh_intf_hdl_t intf_hdl)
{
    intf_t *intf_obj = (intf_t *)intf_hdl;
    usb_desc_intf_t *intf_desc = intf_obj->desc;

    //Print contents and interface descriptor and all its endpoint descriptors
    //Printf interface descriptor
    printf("\tbLength %d\n", intf_desc->bLength);
    printf("\tbDescriptorType %d\n", intf_desc->bDescriptorType);
    printf("\tbInterfaceNumber %d\n", intf_desc->bInterfaceNumber);
    printf("\tbAlternateSetting %d\n", intf_desc->bAlternateSetting);
    printf("\tbNumEndpoints %d\n", intf_desc->bNumEndpoints);
    printf("\tbInterfaceClass 0x%x\n", intf_desc->bInterfaceClass);
    printf("\tbInterfaceSubClass 0x%x\n", intf_desc->bInterfaceSubClass);
    printf("\tbInterfaceProtocol 0x%x\n", intf_desc->bInterfaceProtocol);
    printf("\tiInterface %d\n", intf_desc->iInterface);

    for (int num_ep = 0; num_ep < intf_obj->num_endpoints; num_ep++) {
        print_ep_desc(intf_obj->endpoints[num_ep]->desc);
    }

    return ESP_OK;
}

esp_err_t usbh_print_cfg_desc(const uint8_t *cfg_desc_buff)
{
    USBH_CHECK(cfg_desc_buff != NULL, ESP_ERR_INVALID_ARG);

    //Print config descriptor
    const usb_desc_config_t *cfg_desc = (const usb_desc_config_t *) cfg_desc_buff;
    printf("bLength %d\n", cfg_desc->bLength);
    printf("bDescriptorType %d\n", cfg_desc->bDescriptorType);
    printf("wTotalLength %d\n", cfg_desc->wTotalLength);
    printf("bNumInterfaces %d\n", cfg_desc->bNumInterfaces);
    printf("bConfigurationValue %d\n", cfg_desc->bConfigurationValue);
    printf("iConfiguration %d\n", cfg_desc->iConfiguration);
    printf("bmAttributes 0x%x\n", cfg_desc->bmAttributes);
    printf("bMaxPower %dmA\n", cfg_desc->bMaxPower * 2);
    return ESP_OK;
}

esp_err_t usbh_print_devc_desc(const usb_desc_device_t *devc_desc)
{
    USBH_CHECK(devc_desc != NULL, ESP_ERR_INVALID_ARG);
    printf("bLength %d\n", devc_desc->bLength);
    printf("bDescriptorType %d\n", devc_desc->bDescriptorType);
    printf("bcdUSB %d.%d0\n", ((devc_desc->bcdUSB >> 8) & 0xF), ((devc_desc->bcdUSB >> 4) & 0xF));
    printf("bDeviceClass 0x%x\n", devc_desc->bDeviceClass);
    printf("bDeviceSubClass 0x%x\n", devc_desc->bDeviceSubClass);
    printf("bDeviceProtocol 0x%x\n", devc_desc->bDeviceProtocol);
    printf("bMaxPacketSize0 %d\n", devc_desc->bMaxPacketSize0);
    printf("idVendor 0x%x\n", devc_desc->idVendor);
    printf("idProduct 0x%x\n", devc_desc->idProduct);
    printf("bcdDevice %d.%d0\n", ((devc_desc->bcdDevice >> 8) & 0xF), ((devc_desc->bcdDevice >> 4) & 0xF));
    printf("iManufacturer %d\n", devc_desc->iManufacturer);
    printf("iProduct %d\n", devc_desc->iProduct);
    printf("iSerialNumber %d\n", devc_desc->iSerialNumber);
    printf("bNumConfigurations %d\n", devc_desc->bNumConfigurations);
    return ESP_OK;
}

esp_err_t usbh_set_cur_config(usbh_devc_hdl_t devc_hdl, uint8_t bConfigurationValue, uint32_t timeout_ms)
{
    //Check arguments

    //Check if config number is already set

    //Fetch the desired config descriptor

    //Send the set config request

    return ESP_OK;
}

esp_err_t usbh_get_cur_config(usbh_devc_hdl_t devc_hdl, uint8_t *bConfigurationValue)
{
    USBH_CHECK(devc_hdl != NULL && bConfigurationValue != NULL, ESP_ERR_INVALID_ARG);
    devc_t *devc = (devc_t *)devc_hdl;
    USBH_ENTER_CRITICAL();
    *bConfigurationValue = devc->config.cfg_num;
    USBH_EXIT_CRITICAL();
    return ESP_OK;
}

esp_err_t xfer_len_round_up_to_mps_multiple(usbh_devc_hdl_t device_hdl, int *xfer_size)
{
    USBH_CHECK(device_hdl != NULL && xfer_size != NULL, ESP_ERR_INVALID_ARG);

    devc_t *device = (devc_t *)device_hdl;
    *xfer_size = round_up_to_mps_multiple(*xfer_size, device->info.dflt_pipe_mps);

    return ESP_OK;
}
