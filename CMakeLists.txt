cmake_minimum_required(VERSION 3.16)

set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options(-Wno-error=format)
project(ESPCast)

if(CONFIG_LV_TICK_CUSTOM)
    # LVGL 8 exposes the custom header in Kconfig, but not the time expression.
    # Use the monotonic clock directly instead of waking a task every 2 ms.
    idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
    target_compile_definitions(${lvgl_lib} PUBLIC
        "LV_TICK_CUSTOM_SYS_TIME_EXPR=(esp_timer_get_time()/1000LL)")
endif()
