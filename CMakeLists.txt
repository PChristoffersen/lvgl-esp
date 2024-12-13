

idf_component_register(
    SRC_DIRS src
    INCLUDE_DIRS include
    REQUIRES lvgl esp_lcd
)

idf_build_get_property(build_components BUILD_COMPONENTS)
if(lvgl IN_LIST build_components)
    set(lvgl_name lvgl) # Local component
    set(lvgl_ver $ENV{LVGL_VERSION}) # Get the version from env variable (set from LVGL v9.2)
else()
    set(lvgl_name lvgl__lvgl) # Managed component
    idf_component_get_property(lvgl_ver ${lvgl_name} COMPONENT_VERSION) # Get the version from esp-idf build system
endif()

if(CONFIG_IDF_TARGET_ESP32 OR CONFIG_IDF_TARGET_ESP32S3)
    # Include component libraries, so lvgl component would see lvgl_esp includes
    idf_component_get_property(lvgl_lib ${lvgl_name} COMPONENT_LIB)
    target_include_directories(${lvgl_lib} PRIVATE "include")

    if(CONFIG_LV_ESP_MALLOC_IN_SPIRAM AND CONFIG_LV_USE_BUILTIN_MALLOC)
        target_compile_definitions(${lvgl_lib} PUBLIC CONFIG_LV_ATTRIBUTE_LARGE_RAM_ARRAY=EXT_RAM_BSS_ATTR)
    endif()
endif()
