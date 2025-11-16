idf_component_register(
    SRCS "src/fried_esp_cyd_screen.c" "src/lcd.c" "src/touch.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd driver esp_timer
)

target_compile_definitions(${COMPONENT_LIB} INTERFACE LV_COLOR_16_SWAP=1)