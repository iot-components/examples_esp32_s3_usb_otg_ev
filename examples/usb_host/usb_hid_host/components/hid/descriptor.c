#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "libusb.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/usb_types_stack.h"

typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
} desc_header_t;

#define DESC_HEADER_LENGTH  2
#define USB_MAXENDPOINTS    32
#define USB_MAXINTERFACES   32
#define USB_MAXCONFIG       8

#define TAG "DESC"

#define USB_DESC_ATTR           __attribute__((packed))

/** @defgroup libusb_desc USB descriptors
 * This page details how to examine the various standard USB descriptors
 * for detected devices
 */

#define READ_LE16(p) ((uint16_t)    \
    (((uint16_t)((p)[1]) << 8) |    \
     ((uint16_t)((p)[0]))))

#define READ_LE32(p) ((uint32_t)    \
    (((uint32_t)((p)[3]) << 24) |   \
     ((uint32_t)((p)[2]) << 16) |   \
     ((uint32_t)((p)[1]) <<  8) |   \
     ((uint32_t)((p)[0]))))

static void parse_descriptor(const void *source, const char *descriptor, void *dest)
{
    const uint8_t *sp = source;
    uint8_t *dp = dest;
    char field_type;

    while (*descriptor) {
        field_type = *descriptor++;
        switch (field_type) {
        case 'b':   /* 8-bit byte */
            *dp++ = *sp++;
            break;
        case 'w':   /* 16-bit word, convert from little endian to CPU */
            dp += ((uintptr_t)dp & 1);  /* Align to 16-bit word boundary */

            *((uint16_t *)dp) = READ_LE16(sp);
            sp += 2;
            dp += 2;
            break;
        case 'd':   /* 32-bit word, convert from little endian to CPU */
            dp += 4 - ((uintptr_t)dp & 3);  /* Align to 32-bit word boundary */

            *((uint32_t *)dp) = READ_LE32(sp);
            sp += 4;
            dp += 4;
            break;
        case 'u':   /* 16 byte UUID */
            memcpy(dp, sp, 16);
            sp += 16;
            dp += 16;
            break;
        }
    }
}

static void clear_endpoint(struct libusb_endpoint_descriptor *endpoint)
{
    free((void *)endpoint->extra);
}

static int parse_endpoint(struct libusb_endpoint_descriptor *endpoint, const uint8_t *buffer, int size)
{
    const desc_header_t *header;
    const uint8_t *begin;
    void *extra;
    int parsed = 0;
    int len;

    if (size < DESC_HEADER_LENGTH) {
        ESP_LOGE(TAG, "short endpoint descriptor read %d/%d",
             size, DESC_HEADER_LENGTH);
        return LIBUSB_ERROR_IO;
    }

    header = (const desc_header_t *)buffer;
    if (header->bDescriptorType != LIBUSB_DT_ENDPOINT) {
        ESP_LOGE(TAG, "unexpected descriptor 0x%x (expected 0x%x)",
            header->bDescriptorType, LIBUSB_DT_ENDPOINT);
        return parsed;
    } else if (header->bLength < LIBUSB_DT_ENDPOINT_SIZE) {
        ESP_LOGE(TAG, "invalid endpoint bLength (%u)", header->bLength);
        return LIBUSB_ERROR_IO;
    } else if (header->bLength > size) {
        ESP_LOGW(TAG, "short endpoint descriptor read %d/%u",
              size, header->bLength);
        return parsed;
    }

    if (header->bLength >= LIBUSB_DT_ENDPOINT_AUDIO_SIZE)
        parse_descriptor(buffer, "bbbbwbbb", endpoint);
    else
        parse_descriptor(buffer, "bbbbwb", endpoint);

    buffer += header->bLength;
    size -= header->bLength;
    parsed += header->bLength;

    /* Skip over the rest of the Class Specific or Vendor Specific */
    /*  descriptors */
    begin = buffer;
    while (size >= DESC_HEADER_LENGTH) {
        header = (const desc_header_t *)buffer;
        if (header->bLength < DESC_HEADER_LENGTH) {
            ESP_LOGE(TAG, "invalid extra ep desc len (%u)",
                 header->bLength);
            return LIBUSB_ERROR_IO;
        } else if (header->bLength > size) {
            ESP_LOGW(TAG, "short extra ep desc read %d/%u",
                  size, header->bLength);
            return parsed;
        }

        /* If we find another "proper" descriptor then we're done  */
        if (header->bDescriptorType == LIBUSB_DT_ENDPOINT ||
            header->bDescriptorType == LIBUSB_DT_INTERFACE ||
            header->bDescriptorType == LIBUSB_DT_CONFIG ||
            header->bDescriptorType == LIBUSB_DT_DEVICE)
            break;

        ESP_LOGD(TAG, "skipping descriptor 0x%x", header->bDescriptorType);
        buffer += header->bLength;
        size -= header->bLength;
        parsed += header->bLength;
    }

    /* Copy any unknown descriptors into a storage area for drivers */
    /*  to later parse */
    len = (int)(buffer - begin);
    if (len <= 0)
        return parsed;

    extra = malloc((size_t)len);
    if (!extra)
        return LIBUSB_ERROR_NO_MEM;

    memcpy(extra, begin, len);
    endpoint->extra = extra;
    endpoint->extra_length = len;

    return parsed;
}

static void clear_interface(struct libusb_interface *usb_interface)
{
    int i;

    if (usb_interface->altsetting) {
        for (i = 0; i < usb_interface->num_altsetting; i++) {
            struct libusb_interface_descriptor *ifp =
                (struct libusb_interface_descriptor *)
                usb_interface->altsetting + i;

            free((void *)ifp->extra);
            if (ifp->endpoint) {
                uint8_t j;

                for (j = 0; j < ifp->bNumEndpoints; j++)
                    clear_endpoint((struct libusb_endpoint_descriptor *)
                               ifp->endpoint + j);
            }
            free((void *)ifp->endpoint);
        }
    }
    free((void *)usb_interface->altsetting);
    usb_interface->altsetting = NULL;
}

static int parse_interface(struct libusb_interface *usb_interface, const uint8_t *buffer, int size)
{
    int len;
    int r;
    int parsed = 0;
    int interface_number = -1;
    const desc_header_t *header;
    const usb_intf_desc_t *if_desc;
    struct libusb_interface_descriptor *ifp;
    const uint8_t *begin;

    while (size >= LIBUSB_DT_INTERFACE_SIZE) {
        struct libusb_interface_descriptor *altsetting;

        altsetting = realloc((void *)usb_interface->altsetting,
            sizeof(*altsetting) * (size_t)(usb_interface->num_altsetting + 1));
        if (!altsetting) {
            r = LIBUSB_ERROR_NO_MEM;
            goto err;
        }
        usb_interface->altsetting = altsetting;

        ifp = altsetting + usb_interface->num_altsetting;
        parse_descriptor(buffer, "bbbbbbbbb", ifp);
        if (ifp->bDescriptorType != LIBUSB_DT_INTERFACE) {
            ESP_LOGE(TAG, "unexpected descriptor 0x%x (expected 0x%x)",
                 ifp->bDescriptorType, LIBUSB_DT_INTERFACE);
            return parsed;
        } else if (ifp->bLength < LIBUSB_DT_INTERFACE_SIZE) {
            ESP_LOGE(TAG, "invalid interface bLength (%u)",
                 ifp->bLength);
            r = LIBUSB_ERROR_IO;
            goto err;
        } else if (ifp->bLength > size) {
            ESP_LOGW(TAG, "short intf descriptor read %d/%u",
                 size, ifp->bLength);
            return parsed;
        } else if (ifp->bNumEndpoints > USB_MAXENDPOINTS) {
            ESP_LOGE(TAG, "too many endpoints (%u)", ifp->bNumEndpoints);
            r = LIBUSB_ERROR_IO;
            goto err;
        }

        usb_interface->num_altsetting++;
        ifp->extra = NULL;
        ifp->extra_length = 0;
        ifp->endpoint = NULL;

        if (interface_number == -1)
            interface_number = ifp->bInterfaceNumber;

        /* Skip over the interface */
        buffer += ifp->bLength;
        parsed += ifp->bLength;
        size -= ifp->bLength;

        begin = buffer;

        /* Skip over any interface, class or vendor descriptors */
        while (size >= DESC_HEADER_LENGTH) {
            header = (const desc_header_t *)buffer;
            if (header->bLength < DESC_HEADER_LENGTH) {
                ESP_LOGE(TAG, "invalid extra intf desc len (%u)", header->bLength);
                r = LIBUSB_ERROR_IO;
                goto err;
            } else if (header->bLength > size) {
                ESP_LOGW(TAG, "short extra intf desc read %d/%u", size, header->bLength);
                return parsed;
            }

            /* If we find another "proper" descriptor then we're done */
            if (header->bDescriptorType == LIBUSB_DT_INTERFACE ||
                header->bDescriptorType == LIBUSB_DT_ENDPOINT ||
                header->bDescriptorType == LIBUSB_DT_CONFIG ||
                header->bDescriptorType == LIBUSB_DT_DEVICE)
                break;

            buffer += header->bLength;
            parsed += header->bLength;
            size -= header->bLength;
        }

        /* Copy any unknown descriptors into a storage area for */
        /*  drivers to later parse */
        len = (int)(buffer - begin);
        if (len > 0) {
            void *extra = malloc((size_t)len);

            if (!extra) {
                r = LIBUSB_ERROR_NO_MEM;
                goto err;
            }

            memcpy(extra, begin, len);
            ifp->extra = extra;
            ifp->extra_length = len;
        }

        if (ifp->bNumEndpoints > 0) {
            struct libusb_endpoint_descriptor *endpoint;
            uint8_t i;

            endpoint = calloc(ifp->bNumEndpoints, sizeof(*endpoint));
            if (!endpoint) {
                r = LIBUSB_ERROR_NO_MEM;
                goto err;
            }

            ifp->endpoint = endpoint;
            for (i = 0; i < ifp->bNumEndpoints; i++) {
                r = parse_endpoint(endpoint + i, buffer, size);
                if (r < 0)
                    goto err;
                if (r == 0) {
                    ifp->bNumEndpoints = i;
                    break;
                }

                buffer += r;
                parsed += r;
                size -= r;
            }
        }

        /* We check to see if it's an alternate to this one */
        if_desc = (const usb_intf_desc_t *)buffer;
        if (size < LIBUSB_DT_INTERFACE_SIZE ||
            if_desc->bDescriptorType != LIBUSB_DT_INTERFACE ||
            if_desc->bInterfaceNumber != interface_number)
            return parsed;
    }

    return parsed;
err:
    clear_interface(usb_interface);
    return r;
}

void clear_configuration(struct libusb_config_descriptor *config)
{
    uint8_t i;

    if (config->interface) {
        for (i = 0; i < config->bNumInterfaces; i++)
            clear_interface((struct libusb_interface *)
                    config->interface + i);
    }
    free((void *)config->interface);
    free((void *)config->extra);
}

static int parse_configuration(struct libusb_config_descriptor *config, const uint8_t *buffer, int size)
{
    uint8_t i;
    int r;
    const desc_header_t *header;
    struct libusb_interface *usb_interface;

    if (size < LIBUSB_DT_CONFIG_SIZE) {
        ESP_LOGE(TAG, "short config descriptor read %d/%d",
             size, LIBUSB_DT_CONFIG_SIZE);
        return LIBUSB_ERROR_IO;
    }

    parse_descriptor(buffer, "bbwbbbbb", config);
    if (config->bDescriptorType != LIBUSB_DT_CONFIG) {
        ESP_LOGE(TAG, "unexpected descriptor 0x%x (expected 0x%x)",
             config->bDescriptorType, LIBUSB_DT_CONFIG);
        return LIBUSB_ERROR_IO;
    } else if (config->bLength < LIBUSB_DT_CONFIG_SIZE) {
        ESP_LOGE(TAG, "invalid config bLength (%u)", config->bLength);
        return LIBUSB_ERROR_IO;
    } else if (config->bLength > size) {
        ESP_LOGE(TAG, "short config descriptor read %d/%u",
             size, config->bLength);
        return LIBUSB_ERROR_IO;
    } else if (config->bNumInterfaces > USB_MAXINTERFACES) {
        ESP_LOGE(TAG, "too many interfaces (%u)", config->bNumInterfaces);
        return LIBUSB_ERROR_IO;
    }

    usb_interface = calloc(config->bNumInterfaces, sizeof(*usb_interface));
    if (!usb_interface)
        return LIBUSB_ERROR_NO_MEM;

    config->interface = usb_interface;

    buffer += config->bLength;
    size -= config->bLength;

    for (i = 0; i < config->bNumInterfaces; i++) {
        int len;
        const uint8_t *begin;

        /* Skip over the rest of the Class Specific or Vendor */
        /*  Specific descriptors */
        begin = buffer;
        while (size >= DESC_HEADER_LENGTH) {
            header = (const desc_header_t *)buffer;
            if (header->bLength < DESC_HEADER_LENGTH) {
                ESP_LOGE(TAG, "invalid extra config desc len (%u)", header->bLength);
                r = LIBUSB_ERROR_IO;
                goto err;
            } else if (header->bLength > size) {
                ESP_LOGW(TAG, "short extra config desc read %d/%u", size, header->bLength);
                config->bNumInterfaces = i;
                return size;
            }

            /* If we find another "proper" descriptor then we're done */
            if (header->bDescriptorType == LIBUSB_DT_ENDPOINT ||
                header->bDescriptorType == LIBUSB_DT_INTERFACE ||
                header->bDescriptorType == LIBUSB_DT_CONFIG ||
                header->bDescriptorType == LIBUSB_DT_DEVICE)
                break;

            ESP_LOGD(TAG, "skipping descriptor 0x%x", header->bDescriptorType);
            buffer += header->bLength;
            size -= header->bLength;
        }

        /* Copy any unknown descriptors into a storage area for */
        /*  drivers to later parse */
        len = (int)(buffer - begin);
        if (len > 0) {
            uint8_t *extra = realloc((void *)config->extra,
                         (size_t)(config->extra_length + len));

            if (!extra) {
                r = LIBUSB_ERROR_NO_MEM;
                goto err;
            }

            memcpy(extra + config->extra_length, begin, len);
            config->extra = extra;
            config->extra_length += len;
        }

        r = parse_interface(usb_interface + i, buffer, size);
        if (r < 0)
            goto err;
        if (r == 0) {
            config->bNumInterfaces = i;
            break;
        }

        buffer += r;
        size -= r;
    }

    return size;

err:
    clear_configuration(config);
    return r;
}

int raw_desc_to_config(const uint8_t *buf, int size, struct libusb_config_descriptor **config)
{
    struct libusb_config_descriptor *_config = calloc(1, sizeof(*_config));
    int r;

    if (!_config)
        return LIBUSB_ERROR_NO_MEM;

    r = parse_configuration(_config, buf, size);
    if (r < 0) {
        ESP_LOGE(TAG, "parse_configuration failed with error %d", r);
        free(_config);
        return r;
    } else if (r > 0) {
        ESP_LOGW(TAG, "still %d bytes of descriptor data left", r);
    }

    *config = _config;
    return LIBUSB_SUCCESS;
}

typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bFirstInterface;
    uint8_t bInterfaceCount;
    uint8_t bFunctionClass;
    uint8_t bFunctionSubClass;
    uint8_t bFunctionProtocol;
    uint8_t iFunction;
} USB_DESC_ATTR ifc_assoc_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint16_t bcdUVC;
    uint16_t wTotalLength;
    uint32_t dwClockFrequency;
    uint8_t  bFunctionProtocol;
    uint8_t  bInCollection;
    uint8_t  baInterfaceNr;
} USB_DESC_ATTR vc_interface_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bNumFormats;
    uint16_t wTotalLength;
    uint8_t  bEndpointAddress;
    uint8_t  bFunctionProtocol;
    uint8_t  bmInfo;
    uint8_t  bTerminalLink;
    uint8_t  bStillCaptureMethod;
    uint8_t  bTriggerSupport;
    uint8_t  bTriggerUsage;
    uint8_t  bControlSize;
    uint8_t  bmaControls;

} USB_DESC_ATTR vs_interface_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  iTerminal;
    uint16_t wObjectiveFocalLengthMin;
    uint16_t wObjectiveFocalLengthMax;
    uint16_t wOcularFocalLength;
    uint8_t  bControlSize;
    uint16_t bmControls;
} USB_DESC_ATTR input_terminal_camera_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  iTerminal;
} USB_DESC_ATTR input_terminal_composite_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  iTerminal;
    uint8_t  bControlSize;
    uint8_t  bmControls;
    uint8_t  bTransportModeSize;
    uint8_t  bmTransportModes[5];
} USB_DESC_ATTR input_terminal_media_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bSourceID;
    uint8_t  iTerminal;
} USB_DESC_ATTR output_terminal_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bUnitID;
    uint8_t  bNrInPins;
    uint8_t  baSourceID1;
    uint8_t  baSourceID2;
    uint8_t  iSelector;
} USB_DESC_ATTR selector_unit_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bUnitID;
    uint8_t  bSourceID;
    uint16_t wMaxMultiplier;
    uint8_t  bControlSize;
    uint16_t bmControls;
    uint8_t  iProcessing;
    uint8_t  bmVideoStandards;
} USB_DESC_ATTR processing_unit_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint16_t wMaxTransferSize;
} USB_DESC_ATTR class_specific_endpoint_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bFormatIndex;
    uint8_t  bNumFrameDescriptors;
    uint8_t  bmFlags;
    uint8_t  bDefaultFrameIndex;
    uint8_t  bAspectRatioX;
    uint8_t  bAspectRatioY;
    uint8_t  bmInterlaceFlags;
    uint8_t  bCopyProtect;
} USB_DESC_ATTR vs_format_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bFormatIndex;
    uint8_t  bmCapabilities;
    uint16_t wWidth;
    uint16_t wHeigh;
    uint32_t dwMinBitRate;
    uint32_t dwMaxBitRate;
    uint32_t dwMaxVideoFrameBufSize;
    uint32_t dwDefaultFrameInterval;
    uint8_t  bFrameIntervalType;
    uint32_t dwFrameInterval; // Continuous or discrete ?
    // uint32_t dwMinFrameInterval;
    // uint32_t dwMaxFrameInterval;
    // uint32_t dwFrameIntervalStep;
} USB_DESC_ATTR vs_frame_desc_t;

// Helper struct
typedef struct {
    uint16_t wWidth;
    uint16_t wHeight;
} USB_DESC_ATTR WidthHeight_t;

// Helper struct
typedef struct {
    uint8_t  bNumCompressionPtn;
    uint8_t  bCompression;
} USB_DESC_ATTR Compression_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bEndpointAddress;
    uint8_t  bNumImageSizePatterns;
    uint16_t wWidth;
    uint16_t wHeight;
    uint8_t  bNumCompressionPtn;
    uint8_t  bCompression;
} USB_DESC_ATTR still_image_frame_desc_t;

typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bColorPrimaries;
    uint8_t  bTransferCharacteristics;
    uint8_t  bMatrixCoefficients;
} USB_DESC_ATTR color_format_desc_t;

#define TERMINAL_INPUT_CAMERA_TYPE      0x0201
#define TERMINAL_INPUT_COMPOSITE_TYPE   0x0401
#define ITT_MEDIA_TRANSPORT_INPUT       0x0202

#define CC_VIDEO 0x0E

#define USB_DESC_ASC_SIZE 8

typedef enum {
    CONFIG_DESC = 0x02,
    STRING_DESC = 0x03,
    INTERFACE_DESC = 0x04,
    ENDPOINT_DESC = 0x05,
    INTERFACE_ASSOC_DESC = 0x0B,
    CS_INTERFACE_DESC = 0x24,
    CS_ENDPOINT_DESC = 0x25,
} descriptor_types_t;

typedef enum {
    VC_HEADER = 0x01,
    VC_INPUT_TERMINAL = 0x02,
    VC_OUTPUT_TERMINAL = 0x03,
    VC_SELECTOR_UNIT = 0x04,
    VC_PROCESSING_UNIT = 0x05,
    VS_FORMAT_MJPEG = 0x06,
    VS_FRAME_MJPEG = 0x07,
    VS_STILL_FRAME = 0x03,
    VS_COLORFORMAT = 0x0D,
} descriptor_subtypes_t;

typedef enum {
    SC_VIDEOCONTROL = 1,
    SC_VIDEOSTREAMING = 2,
} interface_sub_class_t;

static interface_sub_class_t interface_sub_class = SC_VIDEOCONTROL;

static void print_config_desc(const uint8_t *buff)
{
    const usb_desc_config_t *cfg_desc = (const usb_desc_config_t *) buff;
    printf("--- Configuration Descriptor ---\n");
    printf("bLength 0x%x\n", cfg_desc->bLength);
    printf("bDescriptorType 0x%x\n", cfg_desc->bDescriptorType);
    printf("wTotalLength %d\n", cfg_desc->wTotalLength);
    printf("bNumInterfaces %d\n", cfg_desc->bNumInterfaces);
    printf("bConfigurationValue %d\n", cfg_desc->bConfigurationValue);
    printf("iConfiguration %d\n", cfg_desc->iConfiguration);
    printf("bmAttributes 0x%x\n", cfg_desc->bmAttributes);
    printf("bMaxPower %dmA\n", cfg_desc->bMaxPower * 2);
}

static const char * get_ep_type(uint8_t bmAttributes)
{
    int type = bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK;

    switch (type)
    {
        case USB_BM_ATTRIBUTES_XFER_CONTROL: return "CTRL";
        case USB_BM_ATTRIBUTES_XFER_ISOC: return "ISOC";
        case USB_BM_ATTRIBUTES_XFER_BULK: return "BULK";
        case USB_BM_ATTRIBUTES_XFER_INT: return "INT";
    }

    return "UNKNOWN";
}

static void print_endpoint_desc(const uint8_t *buff)
{
    const usb_ep_desc_t *ep_desc = (const usb_ep_desc_t *)buff;

    printf("\t\t--- Endpoint Descriptor ---\n");
    printf("\t\tbLength 0x%x\n", ep_desc->bLength);
    printf("\t\tbDescriptorType 0x%x\n", ep_desc->bDescriptorType);
    printf("\t\tbEndpointAddress 0x%x\tEP %d %s\n", ep_desc->bEndpointAddress,
                                                    (ep_desc->bEndpointAddress & 0xF),
                                                    (ep_desc->bEndpointAddress & 0x80) ? "IN" : "OUT");
    printf("\t\tbmAttributes 0x%x\t%s\n", ep_desc->bmAttributes, get_ep_type(ep_desc->bmAttributes));
    printf("\t\twMaxPacketSize %d\n", ep_desc->wMaxPacketSize);
    printf("\t\tbInterval %d\n", ep_desc->bInterval);
}

static void print_cs_endpoint_desc(const uint8_t *buff)
{
    class_specific_endpoint_desc_t *class_desc = (class_specific_endpoint_desc_t *)buff;
    printf("\t\t--- Class-specific Interrupt Endpoint Descriptor ---\n");
    printf("\t\tbLength 0x%x\n", class_desc->bLength);
    printf("\t\tbDescriptorType 0x%x\n", class_desc->bDescriptorType);
    printf("\t\tbDescriptorSubType %d\n", class_desc->bDescriptorSubType);
    printf("\t\twMaxTransferSize %d\n", class_desc->wMaxTransferSize);
}

static void print_interface_desc(const uint8_t *buff)
{
    const usb_intf_desc_t *desc = (const usb_intf_desc_t *)buff;
    printf("\t--- Interface Descriptor ---\n");
    printf("\tbLength 0x%x\n", desc->bLength);
    printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
    printf("\tbInterfaceNumber %d\n", desc->bInterfaceNumber);
    printf("\tbAlternateSetting %d\n", desc->bAlternateSetting);
    printf("\tbNumEndpoints %d\n", desc->bNumEndpoints);
    printf("\tbInterfaceClass 0x%x\n", desc->bInterfaceClass);
    printf("\tbInterfaceSubClass 0x%x\n", desc->bInterfaceSubClass);
    printf("\tbInterfaceProtocol 0x%x\n", desc->bInterfaceProtocol);
    printf("\tiInterface %d\n", desc->iInterface);

    if(desc->bInterfaceClass == CC_VIDEO) {
        if(desc->bInterfaceSubClass == SC_VIDEOCONTROL) {
            // ESP_LOGI("", "SC_VIDEOCONTROL received");
            interface_sub_class = SC_VIDEOCONTROL;
        } else if(desc->bInterfaceSubClass == SC_VIDEOSTREAMING) {
            // ESP_LOGI("", "SC_VIDEOSTREAMING received");
            interface_sub_class = SC_VIDEOSTREAMING;
        }
    }
}

static void print_interface_assoc_desc(const uint8_t *buff)
{
    const ifc_assoc_desc_t *asc_desc = (const ifc_assoc_desc_t *) buff;
    printf("\t--- Interface Association Descriptor ---\n");
    printf("\tbLength 0x%x\n", asc_desc->bLength);
    printf("\tbDescriptorType 0x%x\n", asc_desc->bDescriptorType);
    printf("\tbInterfaceCount %u\n", asc_desc->bInterfaceCount);
    printf("\tbFirstInterface %d\n", asc_desc->bFirstInterface);
    printf("\tbFunctionClass %d\n", asc_desc->bFunctionClass);
    printf("\tbFunctionSubClass %d\n", asc_desc->bFunctionSubClass);
    printf("\tbFunctionProtocol %d\n", asc_desc->bFunctionProtocol);
    printf("\tiFunction 0x%x\n", asc_desc->iFunction);
}


static void print_class_header_desc(const uint8_t *buff)
{
    if(interface_sub_class == SC_VIDEOCONTROL) {
        const vc_interface_desc_t *desc = (const vc_interface_desc_t *) buff;
        printf("\t--- Class-specific VC Interface Descriptor ---\n");
        printf("\tbLength 0x%x\n", desc->bLength);
        printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
        printf("\tbDescriptorSubType %u\n", desc->bDescriptorSubType);
        printf("\tbcdUVC %x\n", desc->bcdUVC);
        printf("\twTotalLength %u\n", desc->wTotalLength);
        printf("\tdwClockFrequency %u\n", desc->dwClockFrequency);
        printf("\tbFunctionProtocol %u\n", desc->bFunctionProtocol);
        printf("\tbInCollection %u\n", desc->bInCollection);
        printf("\tbaInterfaceNr %u\n", desc->baInterfaceNr);
    } else if(interface_sub_class == SC_VIDEOSTREAMING) {
        const vs_interface_desc_t *desc = (const vs_interface_desc_t *) buff;
        printf("\t--- Class-specific VS Interface Descriptor ---\n");
        printf("\tbLength 0x%x\n", desc->bLength);
        printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
        printf("\tbDescriptorSubType %u\n", desc->bDescriptorSubType);
        printf("\tbNumFormats %x\n", desc->bNumFormats);
        printf("\twTotalLength %u\n", desc->wTotalLength);
        printf("\tbEndpointAddress %u\n", desc->bEndpointAddress);
        printf("\tbFunctionProtocol %u\n", desc->bFunctionProtocol);
        printf("\tbmInfo 0x%x\n", desc->bmInfo);
        printf("\tbTerminalLink %u\n", desc->bTerminalLink);
        printf("\tbStillCaptureMethod %u\n", desc->bStillCaptureMethod);
        printf("\tbTriggerSupport %u\n", desc->bTriggerSupport);
        printf("\tbTriggerUsage %u\n", desc->bTriggerUsage);
        printf("\tbControlSize %u\n", desc->bControlSize);
        printf("\tbmaControls 0x%x\n", desc->bmaControls);
    }
}

static void print_vc_input_terminal_desc(const uint8_t *buff)
{
    const input_terminal_camera_desc_t *desc = (const input_terminal_camera_desc_t *) buff;

    const char *type = NULL;

    switch(desc->wTerminalType) {
        case TERMINAL_INPUT_CAMERA_TYPE: type = "Camera"; break;
        case TERMINAL_INPUT_COMPOSITE_TYPE: type = "Composite"; break;
        case ITT_MEDIA_TRANSPORT_INPUT: type = "Media"; break;
        default: printf("!!!!! Unknown Input terminal descriptor !!!!!\n"); return;

    }

    printf("\t--- Input Terminal Descriptor (%s) ---\n", type);
    printf("\tbLength 0x%x\n", desc->bLength);
    printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
    printf("\tbDescriptorSubType %u\n", desc->bDescriptorSubType);
    printf("\tbTerminalID %x\n", desc->bTerminalID);
    printf("\twTerminalType %x\n", desc->wTerminalType);
    printf("\tbAssocTerminal %u\n", desc->bAssocTerminal);
    printf("\tiTerminal %u\n", desc->iTerminal);

    if(desc->wTerminalType == TERMINAL_INPUT_COMPOSITE_TYPE) {
        return;
    } else if(desc->wTerminalType == TERMINAL_INPUT_CAMERA_TYPE) {
        printf("\twObjectiveFocalLengthMin %u\n", desc->wObjectiveFocalLengthMin);
        printf("\twObjectiveFocalLengthMax %u\n", desc->wObjectiveFocalLengthMax);
        printf("\twOcularFocalLength %u\n", desc->wOcularFocalLength);
        printf("\tbControlSize %u\n", desc->bControlSize);
        printf("\tbmControls 0x%x\n", desc->bmControls);
    } else if(desc->wTerminalType == ITT_MEDIA_TRANSPORT_INPUT) {
        const input_terminal_media_desc_t *desc = (const input_terminal_media_desc_t *) buff;
        printf("\tbControlSize %u\n", desc->bControlSize);
        printf("\tbmControls 0x%x\n", desc->bmControls);
        printf("\tbTransportModeSize %u\n", desc->bTransportModeSize);
        printf("\tbmTransportModes 0x%x 0x%x 0x%x 0x%x 0x%x\n",
            desc->bmTransportModes[0],
            desc->bmTransportModes[1],
            desc->bmTransportModes[2],
            desc->bmTransportModes[3],
            desc->bmTransportModes[4]);
    }
}

static void print_vc_output_terminal_desc(const uint8_t *buff)
{
    const output_terminal_desc_t *desc = (const output_terminal_desc_t *) buff;
    printf("\t--- Output Terminal Descriptor ---\n");
    printf("\tbLength 0x%x\n", desc->bLength);
    printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
    printf("\tbDescriptorSubType %u\n", desc->bDescriptorSubType);
    printf("\tbTerminalID %u\n", desc->bTerminalID);
    printf("\twTerminalType %x\n", desc->wTerminalType);
    printf("\tbAssocTerminal %u\n", desc->bAssocTerminal);
    printf("\tbSourceID %u\n", desc->bSourceID);
    printf("\tiTerminal %u\n", desc->iTerminal);
}

static void print_vc_selector_unit_desc(const uint8_t *buff)
{
    const selector_unit_desc_t *desc = (const selector_unit_desc_t *) buff;
    printf("\t--- Selector Unit Descriptor ---\n");
    printf("\tbLength 0x%x\n", desc->bLength);
    printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
    printf("\tbDescriptorSubType %u\n", desc->bDescriptorSubType);
    printf("\tbUnitID %u\n", desc->bUnitID);
    printf("\tbNrInPins %u\n", desc->bNrInPins);
    printf("\tbaSourceID1 %u\n", desc->baSourceID1);
    printf("\tbaSourceID2 %u\n", desc->baSourceID2);
    printf("\tiSelector %u\n", desc->iSelector);
}

static void print_vc_processing_unit_desc(const uint8_t *buff)
{
    const processing_unit_desc_t *desc = (const processing_unit_desc_t *) buff;
    printf("\t--- Processing Unit Descriptor ---\n");
    printf("\tbLength 0x%x\n", desc->bLength);
    printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
    printf("\tbDescriptorSubType %u\n", desc->bDescriptorSubType);
    printf("\tbUnitID %u\n", desc->bUnitID);
    printf("\tbSourceID %u\n", desc->bSourceID);
    printf("\twMaxMultiplier %u\n", desc->wMaxMultiplier);
    printf("\tbControlSize %u\n", desc->bControlSize);
    printf("\tbmControls 0x%x\n", desc->bmControls);
    printf("\tiProcessing %u\n", desc->iProcessing);
    printf("\tbmVideoStandards 0x%x\n", desc->bmVideoStandards);
}

static void print_vs_format_mjpeg_desc(const uint8_t *buff)
{
    const vs_format_desc_t *desc = (const vs_format_desc_t *) buff;
    printf("\t--- VS Format Descriptor ---\n");
    printf("\tbLength 0x%x\n", desc->bLength);
    printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
    printf("\tbDescriptorSubType 0x%x\n", desc->bDescriptorSubType);
    printf("\tbFormatIndex 0x%x\n", desc->bFormatIndex);
    printf("\tbNumFrameDescriptors %u\n", desc->bNumFrameDescriptors);
    printf("\tbmFlags 0x%x\n", desc->bmFlags);
    printf("\tbDefaultFrameIndex %u\n", desc->bDefaultFrameIndex);
    printf("\tbAspectRatioX %u\n", desc->bAspectRatioX);
    printf("\tbAspectRatioY %u\n", desc->bAspectRatioY);
    printf("\tbmInterlaceFlags 0x%x\n", desc->bmInterlaceFlags);
    printf("\tbCopyProtect %u\n", desc->bCopyProtect);
}

static void print_vs_frame_mjpeg_desc(const uint8_t *buff)
{
    const vs_frame_desc_t *desc = (const vs_frame_desc_t *) buff;
    printf("\t--- VS Frame Descriptor ---\n");
    printf("\tbLength 0x%x\n", desc->bLength);
    printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
    printf("\tbDescriptorSubType 0x%x\n", desc->bDescriptorSubType);
    printf("\tbFormatIndex 0x%x\n", desc->bFormatIndex);
    printf("\tbmCapabilities 0x%x\n", desc->bmCapabilities);
    printf("\twWidth %u\n", desc->wWidth);
    printf("\twHeigh %u\n", desc->wHeigh);
    printf("\tdwMinBitRate %u\n", desc->dwMinBitRate);
    printf("\tdwMaxBitRate %u\n", desc->dwMaxBitRate);
    printf("\tdwMaxVideoFrameBufSize %u\n", desc->dwMaxVideoFrameBufSize);
    printf("\tdwDefaultFrameInterval %u\n", desc->dwDefaultFrameInterval);
    printf("\tbFrameIntervalType %u\n", desc->bFrameIntervalType);

    size_t num_of_intervals = (desc->bLength - (sizeof(vs_frame_desc_t) - 4)) / 4;
    uint32_t *interval = (uint32_t*)&desc->dwFrameInterval;
    for (int i = 0; i < num_of_intervals; ++i) {
        printf("\tFrameInterval[%d] %u\n", i, interval[i]);
    }

    // printf("\tdwMinFrameInterval %u\n", desc->dwFrameInterval);
    // printf("\tdwMaxFrameInterval %u\n", desc->dwMaxFrameInterval);
    // printf("\tdwFrameIntervalStep %u\n", desc->dwFrameIntervalStep);
}

static void print_vs_still_frame_desc(const uint8_t *buff)
{
    const still_image_frame_desc_t *desc = (const still_image_frame_desc_t *) buff;
    printf("\t--- VS Still Format Descriptor ---\n");
    printf("\tbLength 0x%x\n", desc->bLength);
    printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
    printf("\tbDescriptorSubType 0x%x\n", desc->bDescriptorSubType);
    printf("\tbEndpointAddress 0x%x\n", desc->bEndpointAddress);
    printf("\tbNumImageSizePatterns 0x%x\n", desc->bNumImageSizePatterns);

    WidthHeight_t *wh = (WidthHeight_t *)&desc->wWidth;
    for (int i = 0; i < desc->bNumImageSizePatterns; ++i, wh++) {
        printf("\t[%d]: wWidth: %u, wHeight: %u\n", i, wh->wWidth, wh->wHeight);
    }

    Compression_t *c = (Compression_t *)wh;
    printf("\tbNumCompressionPtn %u\n", c->bNumCompressionPtn);
    printf("\tbCompression %u\n", c->bCompression);
}

static void print_vs_color_format_desc(const uint8_t *buff)
{
    const color_format_desc_t *desc = (const color_format_desc_t *) buff;
    printf("\t--- VS Color Format Descriptor ---\n");
    printf("\tbLength 0x%x\n", desc->bLength);
    printf("\tbDescriptorType 0x%x\n", desc->bDescriptorType);
    printf("\tbDescriptorSubType 0x%x\n", desc->bDescriptorSubType);
    printf("\tbColorPrimaries 0x%x\n", desc->bColorPrimaries);
    printf("\tbTransferCharacteristics %u\n", desc->bTransferCharacteristics);
    printf("\tbMatrixCoefficients 0x%x\n", desc->bMatrixCoefficients);
}

static void unknown_desc(const desc_header_t *header)
{
    printf(" !!!!! Unknown Descriptor Len: %d Type: %d Subtype: %d !!!!!\n",
        header->bLength, header->bDescriptorType, header->bDescriptorSubtype);
}

static void print_class_specific_desc(const uint8_t *buff)
{
    desc_header_t *header = (desc_header_t *)buff;

    // // VC_OUTPUT_TERMINAL and VS_STILL_FRAME has identical SubType
    // if(header->bDescriptorSubtype == VC_OUTPUT_TERMINAL) {
    //     if(interface_sub_class == SC_VIDEOCONTROL) {
    //         print_vc_output_terminal_desc(buff);
    //     } else {
    //         print_vs_still_frame_desc(buff);
    //     }

    // }

    switch(header->bDescriptorSubtype) {
        case VC_HEADER:
            print_class_header_desc(buff);
            break;
        case VC_INPUT_TERMINAL:
            print_vc_input_terminal_desc(buff);
            break;
        case VC_SELECTOR_UNIT:
            print_vc_selector_unit_desc(buff);
            break;
        case VC_PROCESSING_UNIT:
            print_vc_processing_unit_desc(buff);
            break;
        case VS_FORMAT_MJPEG:
            if(interface_sub_class == SC_VIDEOCONTROL) {
                printf("\t--- Extension Unit Descriptor Unsupported ... skipping ---\n");;
                return;
            }
            print_vs_format_mjpeg_desc(buff);
            break;
        case VS_FRAME_MJPEG:
            print_vs_frame_mjpeg_desc(buff);
            break;
        case VS_COLORFORMAT:
            print_vs_color_format_desc(buff);
            break;
        case VC_OUTPUT_TERMINAL: // same as VS_STILL_FRAME
            if(interface_sub_class == SC_VIDEOCONTROL) {
                print_vc_output_terminal_desc(buff);
            } else {
                print_vs_still_frame_desc(buff);
            }
            break;
        default: unknown_desc(header);
    }
}

void print_cfg_desc(uint8_t *config, size_t size)
{
    static bool run_once = false;
    if(run_once) {
        return;
    }
    run_once = true;

    const uint8_t *buff = (uint8_t *)config;

    while(size > 0) {
        desc_header_t *header = (desc_header_t *)buff;

        switch(header->bDescriptorType) {
            case CONFIG_DESC:           print_config_desc(buff); break;
            case INTERFACE_DESC:        print_interface_desc(buff); break;
            case ENDPOINT_DESC:         print_endpoint_desc(buff); break;
            case INTERFACE_ASSOC_DESC:  print_interface_assoc_desc(buff); break;
            case CS_INTERFACE_DESC:     print_class_specific_desc(buff); break;
            case CS_ENDPOINT_DESC:      print_cs_endpoint_desc(buff); break;
            case STRING_DESC:           printf(" STRING DESCRIPTOR SKIP\n");; break;
            default: unknown_desc(header);
        }

        buff += header->bLength;
        size -= header->bLength;
    }
}
