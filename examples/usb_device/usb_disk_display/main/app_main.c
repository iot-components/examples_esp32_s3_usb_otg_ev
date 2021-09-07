
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_types.h"
#include "sdmmc_cmd.h"
#include "bsp_esp32_s3_usb_otg_ev.h"
#include "tusb_msc.h"
#include "spi_bus.h"
#include "st7789.h"
#include "screen_driver.h"
#include "jpegd2.h"
#include "vidoplayer.h"
#include "esp_spiffs.h"
#include "app.h"

static const char *TAG = "usb_msc_demo";

#ifdef CONFIG_USE_EXTERNAL_SDCARD
#ifdef CONFIG_SDCARD_INTFC_SPI
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#define BOARD_SDCARD_SPI_CS_PIN BOARD_SDCARD_CD
#elif defined(CONFIG_SDCARD_INTFC_SDIO) && defined(SOC_SDMMC_HOST_SUPPORTED)
#define BOARD_SDCARD_SDIO_CLK_PIN BOARD_SDCARD_SDIO_CLK
#define BOARD_SDCARD_SDIO_CMD_PIN BOARD_SDCARD_SDIO_CMD
#define BOARD_SDCARD_SDIO_DO_PIN BOARD_SDCARD_SDIO_DATA0
#define BOARD_SDCARD_SDIO_D1_PIN BOARD_SDCARD_SDIO_DATA1
#define BOARD_SDCARD_SDIO_D2_PIN BOARD_SDCARD_SDIO_DATA2
#define BOARD_SDCARD_SDIO_D3_PIN BOARD_SDCARD_SDIO_DATA3
#define BOARD_SDCARD_SDIO_DATA_WIDTH 4
#include "driver/sdmmc_host.h"
#else
#error "Not supported interface"
#endif
#endif

#define CONFIG_LCD_BUF_WIDTH 240
#define CONFIG_LCD_BUF_HIGHT 48

#define DEMO_SPI_MAX_TRANFER_SIZE (CONFIG_LCD_BUF_WIDTH * CONFIG_LCD_BUF_HIGHT * 2 + 64)
#define PIC_JPEG_MAX_SIZE (15 * 1024)

// Mount path for the partition
static sdmmc_card_t *mount_card = NULL;
const char *disk_path = "/disk";
const char *sd_path = "/sd";
static scr_driver_t s_lcd;
static scr_info_t s_lcd_info;
static bool s_lcd_inited = false;
/***************************************************************************************
 *LCD code to dispaly each frame from usb,
 *the lcd driver comes from esp-iot-solution, for test only,
 *users can implement their lcd driver instead.
*/
static void lcd_screen_clear(scr_driver_t *lcd, int color);

static void lcd_init(void)
{

    spi_bus_handle_t spi_bus = iot_board_get_handle(BOARD_SPI2_ID);

    if (spi_bus == NULL) {
        ESP_LOGE(TAG, "spi_bus2 create failed");
    }

    scr_interface_spi_config_t spi_lcd_cfg = {
        .spi_bus = spi_bus,
        .pin_num_cs = BOARD_LCD_SPI_CS_PIN,
        .pin_num_dc = BOARD_LCD_SPI_DC_PIN,
        .clk_freq = BOARD_LCD_SPI_CLOCK_FREQ,
        .swap_data = true,
    };

    scr_interface_driver_t *iface_drv;
    scr_interface_create(SCREEN_IFACE_SPI, &spi_lcd_cfg, &iface_drv);
    esp_err_t ret = scr_find_driver(BOARD_LCD_TYPE, &s_lcd);

    if (ESP_OK != ret) {
        return;
        ESP_LOGE(TAG, "screen find failed");
    }

    scr_controller_config_t lcd_cfg = {
        .interface_drv = iface_drv,
        .pin_num_rst = BOARD_LCD_SPI_RESET_PIN,
        .pin_num_bckl = BOARD_LCD_SPI_BL_PIN,
        .rst_active_level = 0,
        .bckl_active_level = 1,
        .offset_hor = 0,
        .offset_ver = 0,
        .width = BOARD_LCD_WIDTH,
        .height = BOARD_LCD_HEIGHT,
#ifdef BOARD_LCD_ST7789_DIR_MIRROR
        .rotate = SCR_DIR_TBRL,
#else
        .rotate = SCR_DIR_LRTB,
#endif
    };

    ret = s_lcd.init(&lcd_cfg);

    if (ESP_OK != ret) {
        return;
        ESP_LOGE(TAG, "screen initialize failed");
    }

    s_lcd.get_info(&s_lcd_info);
    ESP_LOGI(TAG, "Screen name:%s | width:%d | height:%d", s_lcd_info.name, s_lcd_info.width, s_lcd_info.height);

#ifdef BOARD_LCD_ST7789_INVERT
    lcd_st7789_set_invert(false);
#endif

    lcd_screen_clear(&s_lcd, COLOR_ESP_BKGD);
    vTaskDelay(pdMS_TO_TICKS(100));
    s_lcd_inited = true;
}

static void lcd_screen_clear(scr_driver_t *lcd, int color)
{
    scr_info_t lcd_info;
    lcd->get_info(&lcd_info);
    uint16_t *buffer = malloc(lcd_info.width * sizeof(uint16_t));

    if (NULL == buffer) {
        for (size_t y = 0; y < lcd_info.height; y++) {
            for (size_t x = 0; x < lcd_info.width; x++) {
                lcd->draw_pixel(x, y, color);
            }
        }
    } else {
        for (size_t i = 0; i < lcd_info.width; i++) {
            buffer[i] = color;
        }

        for (int y = 0; y < lcd_info.height; y++) {
            lcd->draw_bitmap(0, y, lcd_info.width, 1, buffer);
        }

        free(buffer);
    }
}

static bool lcd_write_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
{
    if (!s_lcd_inited) {
        ESP_LOGW(TAG, "lcd not inited");
        return false;
    }

    s_lcd.draw_bitmap(x, y, w, h, (uint16_t *)data);
    return true;
}

/* Function to initialize SPIFFS */
static esp_err_t init_fat(uint8_t if_internal, sdmmc_card_t **card_handle, const char* base_path)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    esp_err_t ret = ESP_FAIL;
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
if(if_internal) {
    // Handle of the wear levelling library instance
    wl_handle_t wl_handle_1 = WL_INVALID_HANDLE;
    ESP_LOGI(TAG, "using internal flash");
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 9,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    ret = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &wl_handle_1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    return ESP_OK;
} else {
    sdmmc_card_t *card;
    ESP_LOGI(TAG, "using external sdcard");
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = CONFIG_DISK_BLOCK_SIZE
    };

#ifdef CONFIG_SDCARD_INTFC_SPI
    ESP_LOGI(TAG, "Using SPI Interface");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = BOARD_SDCARD_SPI_CS_PIN;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(base_path, &host, &slot_config, &mount_config, &card);

#elif defined(CONFIG_SDCARD_INTFC_SDIO) && defined(SOC_SDMMC_HOST_SUPPORTED)
    ESP_LOGI(TAG, "Using SDIO Interface");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    //host.max_freq_khz = SDMMC_FREQ_26M;

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = BOARD_SDCARD_SDIO_CLK_PIN;
    slot_config.cmd = BOARD_SDCARD_SDIO_CMD_PIN;
    slot_config.d0 = BOARD_SDCARD_SDIO_DO_PIN;
    slot_config.d1 = BOARD_SDCARD_SDIO_D1_PIN;
    slot_config.d2 = BOARD_SDCARD_SDIO_D2_PIN;
    slot_config.d3 = BOARD_SDCARD_SDIO_D3_PIN;
    // To use 1-line SD mode, change this to 1:
    slot_config.width = BOARD_SDCARD_SDIO_DATA_WIDTH;
    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    //slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ret = esp_vfs_fat_sdmmc_mount(base_path, &host, &slot_config, &mount_config, &card);
#else
#error "Not supported interface"
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }

        return ret;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    if (card_handle) {
        *card_handle = card;
    }

}
    return ESP_OK;
}

extern esp_err_t start_file_server(const char *base_path);


QueueHandle_t ui_queue = NULL;

typedef enum {
    BTN_CLICK_MENU = 0,
    BTN_CLICK_UP,
    BTN_CLICK_DOWN,
    BTN_CLICK_OK,
} hmi_event_id_t;

typedef struct {
    hmi_event_id_t id;
} hmi_event_t;

static void button_ok_single_click_cb(void *arg)
{
    hmi_event_t item_id = {.id = BTN_CLICK_OK};
    if( ui_queue != NULL ) {
        // Send an uint32_t.  Wait for 10 ticks for space to become
        // available if necessary.
        if( xQueueSend( ui_queue, ( void * ) &item_id, ( TickType_t ) 200) != pdPASS )
        {
            // Failed to post the message, even after 10 ticks.
        }
    }
    ESP_LOGI(TAG, "BTN OK: BUTTON_SINGLE_CLICK");
}

static void button_up_single_click_cb(void *arg)
{
    hmi_event_t item_id = {.id = BTN_CLICK_UP};
    if( ui_queue != NULL ) {
        // Send an uint32_t.  Wait for 10 ticks for space to become
        // available if necessary.
        if( xQueueSend( ui_queue, ( void * ) &item_id, ( TickType_t ) 200) != pdPASS )
        {
            // Failed to post the message, even after 10 ticks.
        }
    }
    ESP_LOGI(TAG, "BTN UP: BUTTON_SINGLE_CLICK");
}

static void button_dw_single_click_cb(void *arg)
{
    hmi_event_t item_id = {.id = BTN_CLICK_DOWN};
    if( ui_queue != NULL ) {
        // Send an uint32_t.  Wait for 10 ticks for space to become
        // available if necessary.
        if( xQueueSend( ui_queue, ( void * ) &item_id, ( TickType_t ) 200) != pdPASS )
        {
            // Failed to post the message, even after 10 ticks.
        }
    }
    ESP_LOGI(TAG, "BTN DW: BUTTON_SINGLE_CLICK");
}

static void button_menu_single_click_cb(void *arg)
{
    hmi_event_t item_id = {.id = BTN_CLICK_MENU};
    if( ui_queue != NULL ) {
        // Send an uint32_t.  Wait for 10 ticks for space to become
        // available if necessary.
        if( xQueueSend( ui_queue, ( void * ) &item_id, ( TickType_t ) 200) != pdPASS )
        {
            // Failed to post the message, even after 10 ticks.
        }
    }
    ESP_LOGI(TAG, "BTN MENU: BUTTON_SINGLE_CLICK");
}

static __NOINIT_ATTR int s_driver_index = -1;

void app_main(void)
{
    const int driver_tail = _app_driver_count;
    const int driver_head = 0;
    printf("````dddddd =%d\n",s_driver_index);
    iot_board_init();
    iot_board_usb_set_mode(USB_DEVICE_MODE);
    iot_board_usb_device_set_power(false, false);

    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_OK_ID), BUTTON_SINGLE_CLICK, button_ok_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_UP_ID), BUTTON_SINGLE_CLICK, button_up_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_DW_ID), BUTTON_SINGLE_CLICK, button_dw_single_click_cb);
    iot_board_button_register_cb(iot_board_get_handle(BOARD_BTN_MN_ID), BUTTON_SINGLE_CLICK, button_menu_single_click_cb);
    /* Initialize file storage */
    ui_queue = xQueueCreate( 1, sizeof(hmi_event_t));
    assert(ui_queue != NULL);
    hmi_event_t current_event;

    if (esp_reset_reason() == ESP_RST_SW && s_driver_index != -1) {
        /* code */
         _app_driver[s_driver_index].init();
        while (xQueueReceive(ui_queue, &current_event, portMAX_DELAY)) {
            if (current_event.id == BTN_CLICK_MENU){
                s_driver_index = -1;
                esp_restart();
            }
        }
    } else {
        s_driver_index = -1;
    }

    esp_vfs_spiffs_conf_t spiffs_config = {
        .base_path              = "/spiffs",
        .partition_label        = NULL,
        .max_files              = 5,
        .format_if_mount_failed = false
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_config));

    vTaskDelay(100 / portTICK_PERIOD_MS);

    /* Initialize lcd driver for display, the driver comes from esp-iot-solution,
    for test only, users can implement their driver for a specified lcd controller*/
    lcd_init();

    /* malloc a buffer for RGB565 data, as 320*240*2 = 153600B,
    here malloc a smaller buffer refresh lcd with steps */
    uint8_t *lcd_buffer = (uint8_t *)heap_caps_malloc(DEMO_SPI_MAX_TRANFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(lcd_buffer != NULL);

    bool led_state = false;
    uint8_t *jpeg_buf = malloc(PIC_JPEG_MAX_SIZE);
    assert(jpeg_buf != NULL);
    char file_name[64] = {0};

    while (1) {
        if(xQueueReceive(ui_queue, &current_event, portMAX_DELAY) != pdTRUE) continue;
        switch (current_event.id) {
            case BTN_CLICK_MENU:
                s_driver_index = -1;
                esp_restart();
                break;
            case BTN_CLICK_UP:
                if (++s_driver_index >= driver_tail)
                s_driver_index = driver_head;
                break;
            case BTN_CLICK_DOWN:
                if (--s_driver_index < driver_head)
                s_driver_index = driver_tail - 1;
                printf("dddddddddddddsssss %d",s_driver_index);
                break;
            case BTN_CLICK_OK:
                if(s_driver_index >= 0 && s_driver_index < _app_driver_count)
                esp_restart();
                break;
            default:
                assert(0);
                break;
        }
        printf("dddddd =%d\n",s_driver_index);
        iot_board_led_all_set_state(led_state);
        led_state = !led_state;
        sprintf(file_name, "/spiffs/icon/%s", _app_driver[s_driver_index].icon_name);

        FILE *fd = fopen(file_name, "r");
        if (fd == NULL) {
            ESP_LOGE(TAG, "open %s filed ", file_name);
            continue;
        }
        int read_bytes = fread(jpeg_buf, 1, PIC_JPEG_MAX_SIZE, fd);
        fclose(fd);
        mjpegdraw(jpeg_buf, read_bytes, lcd_buffer, CONFIG_LCD_BUF_WIDTH, CONFIG_LCD_BUF_HIGHT, lcd_write_bitmap, 240, 240);
        //avi_play("/sd/badapple/badapple_color.avi", lcd_buffer, CONFIG_LCD_BUF_WIDTH, CONFIG_LCD_BUF_HIGHT, lcd_write_bitmap, 240, 240);
        ESP_LOGI(TAG, "file_name: %s, fd: %p, read_bytes: %d, free_heap: %d", file_name, fd, read_bytes, esp_get_free_heap_size());
        vTaskDelay(200);
    }
    free(jpeg_buf);
}
