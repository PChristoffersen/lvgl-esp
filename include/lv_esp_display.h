
#pragma once

#include <esp_lcd_types.h>
#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif


lv_disp_t *lv_esp_display_create(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t panel_handle, uint32_t hor_res, uint32_t ver_res);

#ifdef __cplusplus
}
#endif
