#pragma once

#include <esp_lvgl_port.h>

void fs_init(void);
//value between 0 and 100 (inclusive) (0 for off, 100 for max, 75 is good default)
void fs_set_brightness(uint8_t brightness);
lv_obj_t* fs_new_button(int x, int y, int w, int h, const char* text, lv_event_cb_t cb);

