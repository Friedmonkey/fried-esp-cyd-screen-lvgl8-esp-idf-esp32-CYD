#include "fried_esp_cyd_screen.h"

#include <esp_log.h>
#include <esp_err.h>

#include <lvgl.h>
#include <esp_lvgl_port.h>

#include "lcd.h"
#include "touch.h"

static const char* TAG = "fried_screen";

// internal handles (not exposed)
static esp_lcd_panel_io_handle_t s_lcd_io = NULL;
static esp_lcd_panel_handle_t    s_lcd_panel = NULL;
static esp_lcd_touch_handle_t    s_touch = NULL;
static lv_display_t*             s_lvgl_display = NULL;
static lv_obj_t* 				 s_screen = NULL;

// ---------------------------
// PUBLIC INIT
// ---------------------------
//#define FRIED_ESP_CYD_SCREEN_DEMO

void fs_set_brightness(uint8_t brightness)
{
	lcd_display_brightness_set(brightness);
}

void fs_init(void)
{
    ESP_LOGI(TAG, "Init LCD backlight");
    ESP_ERROR_CHECK(lcd_display_brightness_init());

    ESP_LOGI(TAG, "Init LCD panel");
    ESP_ERROR_CHECK(app_lcd_init(&s_lcd_io, &s_lcd_panel));

    ESP_LOGI(TAG, "Init LVGL");
    s_lvgl_display = app_lvgl_init(s_lcd_io, s_lcd_panel);
    if (!s_lvgl_display) {
        ESP_LOGE(TAG, "LVGL init failed");
        abort();
    }

    ESP_LOGI(TAG, "Init Touch");
    ESP_ERROR_CHECK(touch_init(&s_touch));

    lvgl_port_touch_cfg_t tc = {
        .disp   = s_lvgl_display,
        .handle = s_touch
    };
    lvgl_port_add_touch(&tc);

    // defaults
    lcd_display_brightness_set(75);
    lcd_display_rotate(s_lvgl_display, LV_DISPLAY_ROTATION_90);
	lvgl_port_lock(0);
	s_screen = lv_scr_act();
    lv_obj_set_style_bg_color(s_screen, lv_color_black(), LV_STATE_DEFAULT);

    lvgl_port_unlock();

    ESP_LOGI(TAG, "fried_screen_init done :D");
}


// ---------------------------
// PUBLIC: create button
// ---------------------------
lv_obj_t* fs_new_button(int x, int y, int w, int h,
                                     const char* text, lv_event_cb_t cb)
{
    lvgl_port_lock(0);

    lv_obj_t* btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);

    if (cb)
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    if (text) {
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, text);
        lv_obj_center(lbl);
    }

    lvgl_port_unlock();

    return btn;
}
