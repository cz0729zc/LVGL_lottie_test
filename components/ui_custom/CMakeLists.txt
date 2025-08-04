file(GLOB_RECURSE UI_CUSTOM_SRCS
    "*.c"
)

idf_component_register(
    SRCS ${UI_CUSTOM_SRCS}
    INCLUDE_DIRS "." "guider_customer_fonts" "images"
    REQUIRES lvgl__lvgl esp_rlottie main
)