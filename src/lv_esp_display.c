#include "lv_esp_display.h"

#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <lvgl.h>
#include <lvgl_private.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif


static const char TAG[] = "lv_esp";



#define LVGL_PORT_DEBUG_FLUSH_TIME 0


typedef struct {
    lv_display_t *disp;

    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;
    SemaphoreHandle_t flush_sem;

    #if CONFIG_PM_ENABLE
    esp_pm_lock_handle_t render_pm_lock;
    #endif

    #if LVGL_PORT_DEBUG_FLUSH_TIME
    uint64_t flush_start_ts;
    uint64_t flush_time;
    size_t   flush_sz;
    #endif
} lv_esp_disp_dev_t;




static bool lvgl_port_flush_io_ready_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_esp_disp_dev_t *dev = (lv_esp_disp_dev_t*)user_ctx;
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(dev->flush_sem, &higherPriorityTaskWoken);
    #if LVGL_PORT_DEBUG_FLUSH_TIME
    dev->flush_time = esp_timer_get_time() - dev->flush_start_ts;
    #endif
    return higherPriorityTaskWoken==pdTRUE;
}

static void lvgl_port_flush_wait_callback(lv_display_t * disp)
{
    lv_esp_disp_dev_t *dev = lv_display_get_driver_data(disp);
    xSemaphoreTake(dev->flush_sem, portMAX_DELAY);
    #if LVGL_PORT_DEBUG_FLUSH_TIME
    ESP_LOGI(TAG, "Flushed %6u byte    %6llu us", dev->flush_sz, dev->flush_time);
    #endif
}


static void lvgl_port_flush_callback(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map)
{
    lv_esp_disp_dev_t *dev = lv_display_get_driver_data(disp);

    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    #if CONFIG_LV_COLOR_DEPTH_16
    lv_draw_sw_rgb565_swap(color_map, lv_area_get_size(area));
    #endif

    #if LVGL_PORT_DEBUG_FLUSH_TIME
    dev->flush_start_ts = esp_timer_get_time();
    dev->flush_sz = lv_area_get_size(area)*CONFIG_LV_COLOR_DEPTH/8;
    #endif
    //ESP_LOGI(TAG, "Flush %u,%u -> %ux%u", offsetx1, offsety1, offsetx2-offsetx1 + 1, offsety2-offsety1 + 1);
    esp_lcd_panel_draw_bitmap(dev->panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}



/* ------------------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------------------ */


#if CONFIG_PM_ENABLE

static void display_render_start_cb(lv_event_t * e)
{
    lv_esp_disp_dev_t *dev = lv_event_get_user_data(e);
    esp_pm_lock_acquire(dev->render_pm_lock);
}

static void display_render_ready_cb(lv_event_t * e)
{
    lv_esp_disp_dev_t *dev = lv_event_get_user_data(e);
    esp_pm_lock_release(dev->render_pm_lock);
}

#endif


/* ------------------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------------------ */

lv_disp_t *lv_esp_display_create(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t panel_handle, uint32_t hor_res, uint32_t ver_res)
{
    lv_esp_disp_dev_t *dev = lv_malloc_zeroed(sizeof(lv_esp_disp_dev_t));
    LV_ASSERT_MALLOC(dev);

    dev->flush_sem = xSemaphoreCreateBinary();
    dev->panel_handle = panel_handle;
    dev->io_handle = io_handle;

    lv_disp_t *disp = lv_display_create(hor_res, ver_res);
    LV_ASSERT_NULL(disp);

    lv_display_set_driver_data(disp, dev);

    lv_display_set_flush_cb(disp, lvgl_port_flush_callback);
    lv_display_set_flush_wait_cb(disp, lvgl_port_flush_wait_callback);

    #if CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "lvglRender", &dev->render_pm_lock));
    lv_display_add_event_cb(disp, display_render_start_cb, LV_EVENT_RENDER_START, dev);
    lv_display_add_event_cb(disp, display_render_ready_cb, LV_EVENT_RENDER_READY, dev);
    #endif

    // Register done callback 
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = lvgl_port_flush_io_ready_callback,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, dev);

    ESP_LOGD(TAG, "Display created");

    return disp;
}
