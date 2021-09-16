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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"
#include "soc/usb_types.h"
#include "usb.h"
#include "usb_private.h"

/* -----------------------------------------------------------------------------
----------------------------------- Typedefs -----------------------------------
----------------------------------------------------------------------------- */

// ------------------------------ Object Handles -------------------------------

/**
 * @brief Device Handle
 *
 * - Handle to a connected device that has been opened
 * - Use usbh_devc_open() to get the handle
 */
typedef void * usbh_devc_hdl_t;

/**
 * @brief Interface Handle
 *
 * - Handle to an interface (of a device) that has been claimed
 * - Use usbh_intf_claim() to get the interface handle
 */
typedef void * usbh_intf_hdl_t;

// --------------------------------- Events ------------------------------------

typedef enum {
    USBH_DRVR_EVT_NEW_DEVC = 0,         /**< The new device has been conencted to the host port, enumerated, and configured */
    USBH_DRVR_EVT_PORT_DISCONN,         /**< The host port has disconnected from a device */
    USBH_DRVR_EVT_PORT_OVRCUR,          /**< The host port is overcurrent */
    USBH_DRVR_EVT_ERROR,                /**< The host port has encoutned an error */
} usbh_drvr_evt_t;

typedef enum {
    USBH_DEVC_EVT_SUDDEN_DISCONN = 0,   /**< An opened device has been suddenly disconnected */
    USBH_DEVC_EVT_SUSPEND,              /**< The device has been suspended */
    USBH_DEVC_EVT_RESUMED,              /**< The device has been resumed */
} usbh_devc_evt_t;

typedef enum {
    USBH_INTF_EVT_STALL = 0,            /**< An EP in the interface has been stalled */
    USBH_INTF_EVT_ERR_TRANSACTION,      /**< A transaction has been erroneous after 3 consecutive retries */
    USBH_INTF_EVT_ERR_BNA,              /**< Buffer was not available for a transfer */
    USBH_INTF_EVT_ERR_PKT_BABBLE,       /**< EP packet babble. The EP has returned more bytes in a packet than it's specified MPS */
    USBH_INTF_EVT_ERR_NO_DEVC,          /**< The device has either disconencted or has been unpowered */
    USBH_INTF_EVT_PIPE_FLUSHED,         /**< A pipe has been successfully flushed of all transfer requests */
} usbh_intf_evt_t;

// ------------------------------- Callbacks -----------------------------------

/**
 * @brief Driver callback
 *
 * - Callback that is called when an the USB Host driver has an event
 * - Params
 *      - Bit mask of "driver event flags"
 *
 * @note This callback is called in the context of usbh_handle_events()
 */
typedef void (*usbh_driver_cb_t)(usbh_drvr_evt_t driver_event);

/**
 * @brief Device callback
 *
 * - Callback that is called when an opened device has an event
 *
 */
typedef void (*usbh_devc_cb_t)(usbh_devc_hdl_t device_hdl, usbh_devc_evt_t device_event, void *user_arg);

/**
 * @brief Interface callback
 *
 * - Callback that is called when a claimed interface (or one of the interface's endpoints) has an event
 *
 */
typedef void (*usbh_intf_cb_t)(usbh_intf_hdl_t intf_hdl, usbh_intf_evt_t intf_evt, uint8_t bEndpointAddress, void *user_arg);

/**
 * @brief Transfer request callback
 *
 * - Callback that is called when a transfer request has completed or errors out
 * - Params
 *      - Handle of the transfer request
 *      - Bit mask of "transfer event flags"
 *
 * @note This callback is called in the context of usbh_handle_events()
 */
typedef void (*usbh_xfer_req_cb_t)(urb_t *urb, void *user_arg);    /**< Transfer request callback. */

// --------------------------- Info/Status Objects -----------------------------

/**
 * @brief Device Status
 *
 * - Object containing an opened device's status
 * - Call usbh_get_devc_status()
 *
 * Todo:
 * - Maybe add a status enum (e.g., RUNNING, HALTED, ?)
 * - Some runtime stats (e.g., cumulative bytes transferred)
 */
typedef struct {
    int cur_config;         /**< Current configuration number of the device */
    int num_intf_active;    /**< Number of currently active interfaces of device */
} usbh_devc_status_t;

/**
 * @brief Interface Status
 *
 * - Object continaing the status of a claimed interface
 * - Call usbh_intf_get_status()
 *
 * Todo:
 * - Add some runtime stats (e.g., total bytes transferred, number of bytes in flight)
 */
typedef struct {
    int num_ep_active;      /**< Number of currently active endpoints in interface */
} usbh_intf_status_t;

// --------------------------- Configuration Objects ---------------------------

/**
 * @brief Driver configuration structure
 *
 * Todo:
 * - Add intr_flag to configure driver's interrupt level
 * - Add supported_speeds
 */
typedef struct {
    usbh_driver_cb_t driver_cb;
} usbh_config_t;

// ------------------------------- USB Objects ---------------------------------

/**
 * @brief Object representing a connected/enumerated USB device
 *
 * - Call usbh_devc_get_list() to get a list of connected/enumerated devices
 */
typedef struct {
    int dev_addr;                       /**< The address assigned to the connected device during enumeration */
    const usb_desc_device_t *dev_desc;    /**< The device descriptor of the connected device that is automatically fetched during enumeration */
} usbh_devc_t;

typedef struct {
    usbh_devc_hdl_t devc_hdl;
    uint8_t bEndpointAddress;
    usbh_xfer_req_cb_t callback;
    void *user_arg;
    uint64_t reserved;
    urb_t urb;
} usbh_xfer_req_t;

/* -----------------------------------------------------------------------------
------------------------------- Driver Handling --------------------------------
----------------------------------------------------------------------------- */

/**
 * @brief Install the USB Host driver
 *
 * @param config
 * @return esp_err_t
 */
esp_err_t usbh_install(usbh_config_t *config);

/**
 * @brief Uninstall the USB Host driver
 *
 * @return esp_err_t
 */
esp_err_t usbh_uninstall();

/**
 * @brief Start the USB Host driver
 *
 * - This will power the USB bus
 * - Start detecting connections
 *
 * @return esp_err_t
 */
esp_err_t usbh_start();

/**
 * @brief Stops the USB Host driver
 *
 * @return esp_err_t
 */
esp_err_t usbh_stop();

/**
 * @brief Event handling function that should be repeatedley called
 *
 * This function will block until an event occurs, or when the timeout is
 * reached. The application is expected to call this function repeatedely
 * (usually in a while loop form a separate thread).
 * - The callbacks (driver, interface, transfer request) are run in the context
 *   of this function
 *
 * @note Events should start occurring when the USB host driver is started.
 * Therefore, users are expected to start calling this function as soon as the
 * driver becomes installed.
 *
 * @param[in] timeout_ms Timeout in milliseconds
 * @param[out] events Bit mask of USB Host driver events
 * @return esp_err_t
 */
esp_err_t usbh_handle_events(uint32_t timeout_ms);

/* -----------------------------------------------------------------------------
------------------------------- Device Handling --------------------------------
----------------------------------------------------------------------------- */

/**
 * @brief Gets a list of connected devices
 *
 * @param[out] devc_list
 * @param[out] num_conn_devcs
 * @return esp_err_t
 */
esp_err_t usbh_devc_get_list(usbh_devc_t ***devc_list, int *num_conn_devcs);

/**
 * @brief Free the list of connected devices
 *
 * @param devc_list
 * @return esp_err_t
 */
esp_err_t usbh_devc_free_list(usbh_devc_t **devc_list);

/**
 * @brief Opens a connected device
 *
 * - Will internally allocate resources for this device
 * - Provides a handle to refer to the opended device
 *
 * @param devc
 * @param callback
 * @param user_arg
 * @param[out] devc_hdl
 * @return esp_err_t
 */
esp_err_t usbh_devc_open(usbh_devc_t *devc, usbh_devc_cb_t callback, void *user_arg, usbh_devc_hdl_t *devc_hdl);

/**
 * @brief Close a connected device
 *
 * - Will free the resources used by this device
 *
 * @param devc_hdl Handle of the opened device
 * @return esp_err_t
 */
esp_err_t usbh_devc_close(usbh_devc_hdl_t devc_hdl);


/**
 * @brief Get current status of opened device
 *
 * @note Not really used right now. Will think of more things to add to
 * usbh_devc_status_t later
 *
 * @param devc_hdl Handle of the opened device
 * @param devc_info
 * @return esp_err_t
 */

/**
 * @brief Get current status of opened device
 *
 * @note Not really used right now. Will think of more things to add to
 * usbh_devc_status_t later
 *
 * @param devc_hdl Handle of the opened device
 * @param[out] devc_info Device information object
 * @return esp_err_t
 */
esp_err_t usbh_get_devc_status(usbh_devc_hdl_t devc_hdl, usbh_devc_status_t *devc_info);

/**
 * @brief Sends a reset signal to an opened device
 *
 * @param devc_hdl Handle of the opened device
 * @return esp_err_t
 */
esp_err_t usbh_reset_device(usbh_devc_hdl_t devc_hdl);

/* -----------------------------------------------------------------------------
------------------------------ Interface Handling ------------------------------
----------------------------------------------------------------------------- */

/**
 * @brief Claim an interface of an already configured device
 *
 * @note Users are expected to have used usbh_desc_get_full_config() to check what
 * interfaces are available on the opened device.
 *
 * @param devc_hdl Handle of the opened device
 * @param callback
 * @param[out] intf_hdl
 * @return esp_err_t
 */
esp_err_t usbh_intf_claim(usbh_devc_hdl_t devc_hdl, int interface_number, int alt_interface, usbh_intf_hdl_t *intf_hdl);

/**
 * @brief Release an interface
 *
 * All transfer should already have been completed before realeasing
 *
 * @param intf_hdl
 * @return esp_err_t
 */
esp_err_t usbh_intf_release(usbh_devc_hdl_t devc_hdl, usbh_intf_hdl_t intf_hdl);
/**
 * @brief Flush all transfer requests from a pipe
 *
 * This can to flush all transfer requests that are queued in a pipe.
 *  - If the pipe is stalled, call this after receiving a STALL event to flush the pipe
 *  - If the pipe is still running, call this to stop the current transfer request and flush any pending transfer requests
 *
 * @param intf_hdl
 * @param bEndpointAddress
 * @return esp_err_t
 */
esp_err_t usbh_intf_flush_pipe(usbh_intf_hdl_t intf_hdl, uint8_t bEndpointAddress);

/**
 * @brief Get current status of a claimed interface
 *
 * @note NOt really used, but more stuff will be added to usbh_intf_status_t in the future
 *
 * @param intf_hdl
 * @param intf_status
 * @return esp_err_t
 */
esp_err_t usbh_intf_get_status(usbh_intf_hdl_t intf_hdl, usbh_intf_status_t *intf_status);

/* -----------------------------------------------------------------------------
------------------------------ Transfer Handling ------------------------------
----------------------------------------------------------------------------- */

/* -------------------------------- Async I/O ------------------------------- */

/**
 * @brief Submit a transfer request
 *
 * @note Users should fill the transfer request first before submitting
 *
 * @param xfer_req
 * @return esp_err_t
 */
esp_err_t usbh_xfer_req_submit(usbh_xfer_req_t *xfer_req);

/**
 * @brief Cancel a previously submitted transfer request
 *
 * @todo delete this?
 *
 * @param xfer_req_hdl
 * @return esp_err_t
 */
esp_err_t usbh_xfer_req_cancel(usbh_xfer_req_t *xfer_req);

/* -------------------------------- Sync I/O ------------------------------- */

/**
 * @brief Do a blocking control transfer on a device's default control pipe
 *
 * @note If you need to do a control transfer to another EP that is not the
 * default control endpoint (this is very unlikely), you'll need to use async I/O
 *
 * @param devc_hdl Handle of the opened device
 * @param req
 * @param data
 * @param xfer_len
 * @param timeout_ms
 * @return esp_err_t
 */
esp_err_t usbh_ctrl_xfer(usbh_devc_hdl_t devc_hdl, usb_ctrl_req_t *req, uint8_t *data, uint16_t *xfer_len, uint32_t timeout_ms);

/**
 * @brief Do a blocking bulk transfer
 *
 * @param devc_hdl Handle of the opened device
 * @param bEndpointAddress
 * @param data
 * @param xfer_len
 * @param timeout_ms
 * @return esp_err_t
 */
esp_err_t usbh_bulk_xfer(usbh_devc_hdl_t devc_hdl, uint8_t bEndpointAddress, uint8_t *data, int xfer_len, uint32_t timeout_ms);

/**
 * @brief Do a blocking interrupt transfer
 *
 * Transfer will be done at the next interrupt transfer interval
 *
 * @param devc_hdl Handle of the opened device
 * @param bEndpointAddress
 * @param data
 * @param xfer_len
 * @param timeout_ms
 * @return esp_err_t
 */
esp_err_t usbh_intr_xfer(usbh_devc_hdl_t devc_hdl, uint8_t bEndpointAddress, uint8_t *data, int xfer_len, uint32_t timeout_ms);

/* -----------------------------------------------------------------------------
----------------------------- Chapter 9 Functions ------------------------------
----------------------------------------------------------------------------- */

/* ------------------------------- Descriptors ------------------------------ */

/**
 * @brief Get the device descriptor of an opened device
 *
 * @param devc_hdl Handle of the opened device
 * @param[out] devc_desc Pointer to the device descriptor
 * @return esp_err_t
 */
esp_err_t usbh_desc_get_devc(usbh_devc_hdl_t devc_hdl, const usb_desc_device_t **devc_desc);

/**
 * @brief Get the configuration descriptor of the current active configuration
 *
 * @param devc_hdl
 * @param[out] cfg_desc
 * @return esp_err_t
 */
esp_err_t usbh_desc_get_active_config(usbh_devc_hdl_t devc_hdl, const usb_desc_config_t **cfg_desc);

/**
 * @brief Get a full configuration descriptor of an opened device
 *
 * @param devc_hdl Handle of the opened device
 * @param cfg_idx Configuration index
 * @param[out] cfg_desc_buff Buffer containing the full configuration descriptor
 * @return esp_err_t
 */
esp_err_t usbh_desc_get_full_config(usbh_devc_hdl_t devc_hdl, uint8_t cfg_idx, uint8_t **cfg_desc_buff);

/**
 * @brief Free full configuration descriptor
 *
 * Free the buffer returned from usbh_desc_get_full_config()
 *
 * @param devc_hdl Handle of the opened device
 * @param cfg_desc_buff
 * @return esp_err_t
 */
esp_err_t usbh_desc_free_full_config(usbh_devc_hdl_t devc_hdl, uint8_t *cfg_desc_buff);

/**
 * @brief Get an interface descriptor of the current active configuration
 *
 * @param intf_hdl Claimed interface handle
 * @param[out] intf_desc Interface descriptor
 * @return esp_err_t
 */
esp_err_t usbh_desc_get_intf(usbh_intf_hdl_t intf_hdl, const usb_desc_intf_t **intf_desc);

/**
 * @brief Get an endpoint descriptor of the current active configuration
 *
 * @param intf_hdl Claimed interface handle
 * @param ep_idx Endpoint index
 * @param[out] ep_desc Endpoint descriptor
 * @return esp_err_t
 */
esp_err_t usbh_desc_get_ep(usbh_intf_hdl_t intf_hdl, int ep_idx, const usb_desc_ep_t **ep_desc);

/**
 * @brief Print the contents of a device descriptor
 *
 * @note Use usbh_desc_get_devc() to obtain a device descriptor
 *
 * @param devc_desc Handle of an opened device
 * @return esp_err_t
 */
esp_err_t usbh_print_devc_desc(const usb_desc_device_t *devc_desc);

/**
 * @brief Print the contents of a full configuration descriptor
 *
 * @note Use usbh_desc_get_full_config() to obtain a full configuration descriptor
 *
 * @param cfg_desc_buff Data buffer containing the full configuration descriptor
 * @return esp_err_t
 */
esp_err_t usbh_print_cfg_desc(const uint8_t *cfg_desc_buff);

/**
 * @brief Print Interface descriptor and its endpoint descriptors
 *
 * @note Use usbh_intf_claim() to obtain interface handle
 *
 * @param intf_hdl USB host Interface handle
 * @return ESP_OK Success
 */
esp_err_t usbh_print_intf_desc(usbh_intf_hdl_t intf_hdl);

/* ------------------------------- Requests ------------------------------ */

/**
 * @brief Sets the active configuration for the device
 *
 * If the configuration is changed by this function
 * @note This will cause the driver to save send control transfer to set and then
 * save the corresponding config decriptor for the device
 * @note Setting 0 will return the device to the addressed state
 *
 * @param devc_hdl Handle of the opened device
 * @param config_num
 * @return esp_err_t
 */
esp_err_t usbh_set_cur_config(usbh_devc_hdl_t devc_hdl, uint8_t bConfigurationValue, uint32_t timeout_ms);

/**
 * @brief Get current configuration number for the device
 *
 * @note If 0 is returned, it means the device has not yet been configured
 *
 * @param devc_hdl Handle of the opened device
 * @param config_num
 * @return esp_err_t
 */
esp_err_t usbh_get_cur_config(usbh_devc_hdl_t devc_hdl, uint8_t *bConfigurationValue);

/**
 * @brief As buffer size of IN transaction has to be multiple of MPS (Maximum Packet Size),
 *        this function can be used to obtain sufficient size.
 *
 * @param[in]    device_hdl  Handle to opened device
 * @param[inout] xfer_size   Transfer size to be rounded up to msp multiple
 *
 * @return
 *             - ESP_OK on success.
 *             - ESP_ERR_INVALID_ARG when invalid (NULL) parameter is provided.
 */
esp_err_t xfer_len_round_up_to_mps_multiple(usbh_devc_hdl_t device_hdl, int *xfer_size);

#ifdef __cplusplus
}
#endif
