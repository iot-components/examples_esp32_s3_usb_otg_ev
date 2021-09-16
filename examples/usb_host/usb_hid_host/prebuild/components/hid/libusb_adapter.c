#include "libusb.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_pthread.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb.h"
// #include "descriptor.h"

#define DRIVER_TAG  "USB DRIVER"
#define DEVICE_TAG  "USB DEVICE"

#define DEVICE_CONNECT_TIMEOUT  20000
#define EVENT_QUEUE_LEN         4

typedef struct
{
    struct libusb_transfer *libusb;
    usbh_xfer_req_t req;
} async_transfer_t;

static TaskHandle_t handler_task;
static QueueHandle_t event_queue;

static int esp_to_libusb_error(esp_err_t err)
{
    switch (err) {
        case ESP_ERR_TIMEOUT:   return LIBUSB_ERROR_TIMEOUT;
        case ESP_ERR_NO_MEM:    return LIBUSB_ERROR_NO_MEM;
        case ESP_FAIL:          return LIBUSB_ERROR_PIPE;
        case ESP_OK:            return LIBUSB_SUCCESS;
        default:                return LIBUSB_ERROR_OTHER;
    }
}

static void driver_cb(usbh_drvr_evt_t driver_event)
{
    switch (driver_event) {
        case USBH_DRVR_EVT_NEW_DEVC:
            ESP_LOGI(DRIVER_TAG, "Device connected");
            assert(xQueueSend(event_queue, &driver_event, 0) == pdTRUE);
            break;
        case USBH_DRVR_EVT_PORT_OVRCUR:
            ESP_LOGE(DRIVER_TAG, "Overcurrent");
            break;
        case USBH_DRVR_EVT_ERROR:
            ESP_LOGE(DRIVER_TAG, "Error");
            break;
        default:
            abort();
            break;
        }
}

static void device_cb(usbh_devc_hdl_t device_hdl, usbh_devc_evt_t device_event, void *user_arg)
{
    switch (device_event) {
    case USBH_DEVC_EVT_SUDDEN_DISCONN:
        ESP_LOGI(DEVICE_TAG, "Disconnected");
        break;
    case USBH_DEVC_EVT_SUSPEND:
        ESP_LOGI(DEVICE_TAG, "Suspended");
        break;
    case USBH_DEVC_EVT_RESUMED:
        ESP_LOGI(DEVICE_TAG, "Resumed");
        break;
    }
}

static void event_handler_task()
{
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    while (1) {
        usbh_handle_events(100);
    }
}

int libusb_init(libusb_context **ctx)
{
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&cfg);

    event_queue = xQueueCreate(sizeof(usbh_drvr_evt_t), EVENT_QUEUE_LEN);
    BaseType_t task_created = xTaskCreatePinnedToCore(
        event_handler_task, "usb_hanfler_task", 4096, NULL, 10, &handler_task, 0);
    if (!task_created || event_queue == NULL) {
        goto cleanup;
    }

    usbh_config_t config = {.driver_cb = driver_cb };
    if ( usbh_install(&config) != ESP_OK ) {
        goto cleanup;
    }

    xTaskNotifyGive(handler_task);
    usbh_start();

    usbh_drvr_evt_t event;
    uint32_t timeout = pdMS_TO_TICKS(DEVICE_CONNECT_TIMEOUT);
    BaseType_t event_received = xQueueReceive(event_queue, &event, timeout);

    if(!event_received || event !=  USBH_DRVR_EVT_NEW_DEVC) {
        ESP_LOGD("DRIVER_TAG", "Device not detected within %d ms", DEVICE_CONNECT_TIMEOUT);
    }

    *ctx = (libusb_context *)2;
    return LIBUSB_SUCCESS;

cleanup:
    if (event_queue) { 
        vQueueDelete(event_queue); 
    }
    if (handler_task) { 
        vTaskDelete(handler_task); 
    }
    return LIBUSB_ERROR_NO_MEM;
}

void libusb_exit(libusb_context *ctx)
{
    if (event_queue) {
        vQueueDelete(event_queue); 
    }
    if (handler_task) {
        vTaskDelete(handler_task);
    }
    // usbh_uninstall();
}


// Return number of devices and NULL terminated list of device handles
// Unlike in libusb where descriptors can be retrevied from device without
// opening it, host driver requires to pass handle to opened device 
// into desc_get_* functions. Therefore, handle to opened device is returned.
ssize_t libusb_get_device_list(libusb_context *ctx, libusb_device ***list)
{
    static libusb_device *opened_devices[2] = { NULL, NULL };
    usbh_devc_t **device_list = NULL;
    int device_count = 0;
    esp_err_t err;

    if (usbh_devc_get_list(&device_list, &device_count) != ESP_OK) {
        return LIBUSB_ERROR_NO_DEVICE;
    }

    err = usbh_devc_open(device_list[0], device_cb, NULL, (usbh_devc_hdl_t)&opened_devices[0]);
    if (err) {
        return esp_to_libusb_error(err);
    }

    usbh_devc_free_list(device_list);

    *list = opened_devices;
    return device_count;
}

void libusb_free_device_list(libusb_device **list, int unref_devices)
{
    // device list is released in libusb_get_device_list after use
}

int libusb_open(libusb_device *dev, libusb_device_handle **dev_handle)
{
    *dev_handle = (libusb_device_handle *)dev;
    return 0;
}

void libusb_close(libusb_device_handle *dev_handle)
{
    usbh_devc_close((usbh_devc_hdl_t)dev_handle);
}


void libusb_free_transfer(struct libusb_transfer *transfer)
{
    free(transfer);
}

struct libusb_transfer *libusb_alloc_transfer(int iso_packets)
{
	assert(iso_packets >= 0);

	return calloc(1, sizeof(struct libusb_transfer) +
                     sizeof(struct libusb_iso_packet_descriptor) * iso_packets);
}

static enum libusb_transfer_status eps_to_libusb_status(usb_transfer_status_t esp_status)
{
    switch(esp_status) {
        case USB_TRANSFER_STATUS_COMPLETED: return LIBUSB_TRANSFER_COMPLETED;
        case USB_TRANSFER_STATUS_TIMED_OUT: return LIBUSB_TRANSFER_TIMED_OUT;
        case USB_TRANSFER_STATUS_CANCELED: return LIBUSB_TRANSFER_CANCELLED;
        case USB_TRANSFER_STATUS_NO_DEVICE: return LIBUSB_TRANSFER_NO_DEVICE;
        case USB_TRANSFER_STATUS_OVERFLOW:  return LIBUSB_TRANSFER_OVERFLOW;
        case USB_TRANSFER_STATUS_STALL:     return LIBUSB_TRANSFER_STALL;
        default: return LIBUSB_TRANSFER_ERROR;
    }
}

void transfer_cb(urb_t *urb, void *user_arg)
{
    async_transfer_t *async_transfer = (async_transfer_t *)user_arg;
    struct libusb_transfer *transfer = async_transfer->libusb;
    size_t isoc_actual_length = 0;

    if(urb->transfer.status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGE("", "Transfer failed with status: %d", urb->transfer.status);
    }

    for(int i = 0; i < urb->transfer.num_isoc_packets; i++) {
        transfer->iso_packet_desc[i].actual_length = urb->transfer.isoc_packet_desc[i].actual_num_bytes;
        transfer->iso_packet_desc[i].status = eps_to_libusb_status(urb->transfer.isoc_packet_desc[i].status);

        if(transfer->iso_packet_desc[i].status == LIBUSB_TRANSFER_COMPLETED) {
            isoc_actual_length += transfer->iso_packet_desc[i].actual_length;
        }
    }

    transfer->status = eps_to_libusb_status(urb->transfer.status);
    transfer->actual_length = urb->transfer.num_isoc_packets ? isoc_actual_length : urb->transfer.actual_num_bytes;

    transfer->callback(transfer);

    free(async_transfer);
}

static esp_err_t realocate_dma_capable(struct libusb_transfer *transfer)
{
    uint8_t *new_buffer = NULL;
    int bytes_to_alloc = transfer->length;

    // As libusb uses preallocated array for interrupt transfer, we only change pointer to
    // transmit buffer to be DMA capable, without deallocating provided buffer.
    if(transfer->type == LIBUSB_TRANSFER_TYPE_INTERRUPT) {
        // new_buffer = heap_caps_malloc(transfer->length, MALLOC_CAP_DMA);
        new_buffer = transfer->buffer;
    } else {
        xfer_len_round_up_to_mps_multiple((usbh_devc_hdl_t)transfer->dev_handle, &bytes_to_alloc);
        new_buffer = heap_caps_realloc(transfer->buffer, bytes_to_alloc, MALLOC_CAP_DMA);
    }
    if(new_buffer == NULL) {
        ESP_LOGE("DMA capable alloc failed", "availabele: %d, contiguous: %u, required: %d", 
        heap_caps_get_total_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block( MALLOC_CAP_DMA ), bytes_to_alloc);
        return ESP_ERR_NO_MEM;
    }
    transfer->buffer = new_buffer;
    return ESP_OK;
}

int libusb_submit_transfer(struct libusb_transfer *transfer)
{
    if(realocate_dma_capable(transfer) != ESP_OK) {
        return ESP_ERR_NO_MEM;
    }

    size_t num_iso_packets = transfer->num_iso_packets;
    size_t request_size = sizeof(async_transfer_t) +
                          sizeof(usb_isoc_packet_desc_t) * num_iso_packets;

    async_transfer_t *async_transfer = calloc(1, request_size);
    if(async_transfer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    usbh_xfer_req_t *request =  &async_transfer->req;
    request->devc_hdl = (usbh_devc_hdl_t) transfer->dev_handle;
    request->bEndpointAddress = transfer->endpoint;
    request->callback = transfer_cb;
    request->user_arg = async_transfer;
    usb_transfer_dummy_t *transfer_dummy = (usb_transfer_dummy_t *)&request->urb.transfer;
    transfer_dummy->data_buffer= transfer->buffer;
    transfer_dummy->num_isoc_packets = num_iso_packets;
    request->urb.transfer.num_bytes = transfer->length;
    request->urb.transfer.timeout = transfer->timeout;
    async_transfer->libusb = transfer;

    for(int i = 0; i < num_iso_packets; i++) {
        request->urb.transfer.isoc_packet_desc[i].num_bytes = transfer->iso_packet_desc[i].length;
    }

    esp_err_t err = usbh_xfer_req_submit(request);

    if(err) {
        return err == ESP_ERR_INVALID_ARG ? LIBUSB_ERROR_INVALID_PARAM : LIBUSB_ERROR_PIPE;
    }

    return LIBUSB_SUCCESS;
}

int libusb_cancel_transfer(struct libusb_transfer *transfer)
{
    return 0;
}

int libusb_control_transfer(libusb_device_handle *dev_handle,
                            uint8_t bmRequestType,
                            uint8_t bRequest,
                            uint16_t wValue,
                            uint16_t wIndex,
                            unsigned char *data,
                            uint16_t wLength,
                            unsigned int timeout)
{
    usb_ctrl_req_t req = {
        .bRequestType = bmRequestType,
        .bRequest = bRequest,
        .wValue = wValue,
        .wIndex = wIndex,
        .wLength = wLength,
    };

    esp_err_t err = usbh_ctrl_xfer(dev_handle, &req, data, &wLength, timeout);

    return err ? esp_to_libusb_error(err) : wLength;

}

int libusb_get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc)
{
    const usb_desc_device_t *devc_desc;
    if (usbh_desc_get_devc((usbh_devc_hdl_t)dev, &devc_desc) != ESP_OK) {
        return LIBUSB_ERROR_OTHER;
    }

    desc->bLength =         devc_desc->bLength;
    desc->bDescriptorType = devc_desc->bDescriptorType;
    desc->bcdUSB =          devc_desc->bcdUSB;
    desc->bDeviceClass =    devc_desc->bDeviceClass;
    desc->bDeviceSubClass = devc_desc->bDeviceSubClass;
    desc->bDeviceProtocol = devc_desc->bDeviceProtocol;
    desc->bMaxPacketSize0 = devc_desc->bMaxPacketSize0;
    desc->idVendor =        devc_desc->idVendor;
    desc->idProduct =       devc_desc->idProduct;
    desc->bcdDevice =       devc_desc->bcdDevice;
    desc->iManufacturer =   devc_desc->iManufacturer;
    desc->iProduct =        devc_desc->iProduct;
    desc->iSerialNumber =   devc_desc->iSerialNumber;
    desc->bNumConfigurations = devc_desc->bNumConfigurations;

    return LIBUSB_SUCCESS;
}

void clear_configuration(struct libusb_config_descriptor *config);
int raw_desc_to_config(const uint8_t *buf, int size, struct libusb_config_descriptor **config);
void print_cfg_desc(uint8_t *config, size_t size);

int libusb_get_config_descriptor(libusb_device *dev,
                                 uint8_t config_index,
                                 struct libusb_config_descriptor **config)
{
    uint8_t *data;
    if (usbh_desc_get_full_config((usbh_devc_hdl_t)dev, config_index, &data) != ESP_OK) {
        return LIBUSB_ERROR_OTHER;
    }

    size_t size = ((const usb_desc_config_t *)data)->wTotalLength;
    int res = raw_desc_to_config(data, size, config);
    // usbh_desc_free_full_config((usbh_devc_hdl_t)dev, data);
    print_cfg_desc(data, (*config)->wTotalLength);

    return res;
}

void libusb_free_config_descriptor(struct libusb_config_descriptor *config)
{
    clear_configuration(config);
}


int libusb_get_string_descriptor_ascii(libusb_device_handle *dev_handle,
                                       uint8_t desc_index,
                                       unsigned char *data,
                                       int length)
{
    usb_ctrl_req_t req;
    uint16_t bytes_received;

    USB_CTRL_REQ_INIT_GET_STR_DESC(&req, desc_index, length);

    esp_err_t err = usbh_ctrl_xfer(dev_handle, &req, data, &bytes_received, 1000);

    return err ? esp_to_libusb_error(err) : bytes_received;
}

int libusb_get_ss_endpoint_companion_descriptor(libusb_context *ctx,
        const struct libusb_endpoint_descriptor *endpoint,
        struct libusb_ss_endpoint_companion_descriptor **ep_comp)
{
    return 0;
}

void libusb_free_ss_endpoint_companion_descriptor(struct libusb_ss_endpoint_companion_descriptor *ep_comp)
{

}

libusb_device *libusb_ref_device(libusb_device *dev)
{
    return dev;
}

void libusb_unref_device(libusb_device *dev)
{

}

static usbh_intf_hdl_t intf_handle[5];

int libusb_claim_interface(libusb_device_handle *dev_handle, int interface_number)
{
    usbh_devc_hdl_t dev = (usbh_devc_hdl_t)dev_handle;
    
    // Alternate interface will be clamed in libusb_set_interface_alt_setting function, as libusb
    // only support claming interface without alternate settings.
    esp_err_t err = usbh_intf_claim(dev, interface_number, -1, &intf_handle[interface_number]);
    return esp_to_libusb_error(err);
}

int libusb_release_interface(libusb_device_handle *dev_handle, int interface_number)
{
    esp_err_t err = usbh_intf_release((usbh_devc_hdl_t)dev_handle, intf_handle[interface_number]);
    return esp_to_libusb_error(err);
}

int libusb_set_interface_alt_setting(libusb_device_handle *dev_handle,
                                     int interface_number,
                                     int alternate_setting)
{
    usb_ctrl_req_t req;
    uint16_t bytes_received;
    usbh_devc_hdl_t dev = (usbh_devc_hdl_t)dev_handle;
    usbh_intf_hdl_t intf = intf_handle[interface_number];

    // Release previously clamed interface and claim again with provided alternate settings
    if(intf) {
        usbh_intf_release(dev, intf);
        if( usbh_intf_claim(dev, interface_number, alternate_setting, &intf) ) {
            return LIBUSB_ERROR_NO_MEM;
        }
    }

    USB_CTRL_REQ_INIT_SET_INTERFACE(&req, interface_number, alternate_setting);
    esp_err_t err = usbh_ctrl_xfer(dev_handle, &req, NULL, &bytes_received, 1000);
    return esp_to_libusb_error(err);
}

int libusb_attach_kernel_driver(libusb_device_handle *dev_handle, int interface_number)
{
    return 0;
}

int libusb_detach_kernel_driver(libusb_device_handle *dev_handle, int interface_number)
{
    return 0;
}

int libusb_handle_events_completed(libusb_context *ctx, int *completed)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return 0;
}


// ------------------- UNIMPLEMENTED --------------------------

uint8_t libusb_get_bus_number(libusb_device *dev)
{
    return 1;
}

uint8_t libusb_get_device_address(libusb_device *dev)
{
    return 1; //selected_devc->dev_addr;
}

int libusb_handle_events(libusb_context *ctx)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return 0;
}

int libusb_get_active_config_descriptor(libusb_device *dev, struct libusb_config_descriptor **config)
{
    return libusb_get_config_descriptor(dev, 0, config);
}

int libusb_kernel_driver_active(libusb_device_handle *dev_handle, int interface_number)
{
    return 0;
}

int libusb_interrupt_transfer(
    libusb_device_handle *dev_handle,
	unsigned char endpoint,
    unsigned char *data,
    int length,
	int *actual_length,
    unsigned int timeout)
{
    return 0;
}





void _pthread_cleanup_push (struct _pthread_cleanup_context *_context, 
                            void (*_routine)(void *), void *_arg)
{

}

void _pthread_cleanup_pop (struct _pthread_cleanup_context *_context, int _execute)
{

}