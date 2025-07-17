idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_lcd teriyakigod__esp_lcd_st7735 esp_lvgl_port lvgl__lvgl esp_rlottie)