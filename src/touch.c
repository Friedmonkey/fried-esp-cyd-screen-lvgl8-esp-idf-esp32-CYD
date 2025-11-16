#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_lcd_touch.h>
#include <esp_lcd_touch_xpt2046.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "hardware.h"
#include "touch.h"

uint16_t fried_map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//static uint16_t map(uint16_t n, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
//{
//    uint16_t value = (n - in_min) * (out_max - out_min) / (in_max - in_min);
//    return (value < out_min) ? out_min : ((value > out_max) ? out_max : value);
//}

#define LVGL_WIDTH  320  // what LVGL thinks is width??????????
//i dont even fucking now anymore it seems the height 
// (or width depending on orientation??) is halved from the origional
// but the width isnt?? also i tought lvgl tought my ratio was 320*480 so thats why i have 480
#define LVGL_HEIGHT 480/2  // what LVGL thinks is height?????????!!!!?!?!?!?!?!?!????????????????????????????????????????????!!!!???!?!?!?!?!??!?!?!?!?!??!
//fuck it we ball, it works.

//after copying my code to a new project it seems i have to do /2 instead of /4 so it would've worked here all this time????
//so it IS a config issue after all it seems




//static void process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
//{
//	ESP_LOGI("RAW_TOUCH", "X:%d Y:%d", *x, *y);
//	//*x = fried_map(*x, TOUCH_X_RES_MIN, TOUCH_X_RES_MAX, 1, LCD_V_RES);
//    //*y = fried_map(*y, TOUCH_Y_RES_MIN, TOUCH_Y_RES_MAX, 1, LCD_H_RES);
//	*x = fried_map(*x, TOUCH_X_RES_MIN, TOUCH_X_RES_MAX, 0, LVGL_HEIGHT - 1);
//	*y = fried_map(*y, TOUCH_Y_RES_MIN, TOUCH_Y_RES_MAX, 0, LVGL_WIDTH - 1);
//
//	ESP_LOGI("MAPPED_TOUCH", "X:%d Y:%d", *x, *y);
//}

//#define FRIED_ESP_CYD_SCREEN_DEBUG

static void process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
#ifdef FRIED_ESP_CYD_SCREEN_DEBUG
    ESP_LOGI("RAW_TOUCH", "X:%d Y:%d", *x, *y);
#endif
    // Map and invert vertical (X) to fix up/down
    uint16_t temp_x = fried_map(*x, TOUCH_X_RES_MIN, TOUCH_X_RES_MAX, 0, LVGL_HEIGHT - 1);
    *x = (LVGL_HEIGHT - 1) - temp_x;

    // Horizontal (Y) stays as is
    *y = fried_map(*y, TOUCH_Y_RES_MIN, TOUCH_Y_RES_MAX, 0, LVGL_WIDTH - 1);

#ifdef FRIED_ESP_CYD_SCREEN_DEBUG
    ESP_LOGI("MAPPED_TOUCH", "X:%d Y:%d", *x, *y);
#endif
}




esp_err_t fried_touch_init(esp_lcd_touch_handle_t *tp)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    const esp_lcd_panel_io_spi_config_t tp_io_config = { .cs_gpio_num = TOUCH_CS,
        .dc_gpio_num = TOUCH_DC,
        .spi_mode = 0,
        .pclk_hz = TOUCH_CLOCK_HZ,
        .trans_queue_depth = 3,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = { .dc_low_on_data = 0, .octal_mode = 0, .sio_mode = 0, .lsb_first = 0, .cs_high_active = 0 } };

    static const int SPI_MAX_TRANSFER_SIZE = 32768;
    const spi_bus_config_t buscfg_touch = { .mosi_io_num = TOUCH_SPI_MOSI,
        .miso_io_num = TOUCH_SPI_MISO,
        .sclk_io_num = TOUCH_SPI_CLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .data4_io_num = GPIO_NUM_NC,
        .data5_io_num = GPIO_NUM_NC,
        .data6_io_num = GPIO_NUM_NC,
        .data7_io_num = GPIO_NUM_NC,
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM };

    esp_lcd_touch_config_t tp_cfg = {.x_max = LCD_H_RES,
                                   .y_max = LCD_V_RES,
                                   .rst_gpio_num = TOUCH_RST,
                                   .int_gpio_num = TOUCH_IRQ,
                                   .levels = {.reset = 0, .interrupt = 0},
                                   .flags =
                                       {
                                           .swap_xy = false,
                                           .mirror_x = LCD_MIRROR_X,
                                           .mirror_y = LCD_MIRROR_Y,
                                       },
                                   .process_coordinates = process_coordinates,
                                   .interrupt_callback = NULL};

    ESP_ERROR_CHECK(spi_bus_initialize(TOUCH_SPI, &buscfg_touch, SPI_DMA_CH_AUTO));

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TOUCH_SPI, &tp_io_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, tp));

    return ESP_OK;
}