#include "lv_esp.h"

#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <rom/ets_sys.h>

#include <lvgl.h>
#include <lvgl_esp.h>
#include <lvgl_private.h>

static const char TAG[] = "lvgl_esp";


static bool g_initialized = false;



static uint32_t lv_esp_tick_get()
{
    return pdTICKS_TO_MS(xTaskGetTickCount());
}


static void lv_esp_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}


#if LV_USE_LOG
static void lv_esp_log(lv_log_level_t level, const char * buf)
{
    static const char TAG[] = "lvgl";
    switch (level) {
        #if LV_LOG_LEVEL <= LV_LOG_LEVEL_TRACE
        case LV_LOG_LEVEL_TRACE:
            ESP_LOGD(TAG, "%s", buf);
            break;
        #endif
        #if LV_LOG_LEVEL <= LV_LOG_LEVEL_INFO
        case LV_LOG_LEVEL_INFO:
            ESP_LOGI(TAG, "%s", buf);
            break;
        #endif
        #if LV_LOG_LEVEL <= LV_LOG_LEVEL_WARN
        case LV_LOG_LEVEL_WARN:
            ESP_LOGW(TAG, "%s", buf);
            break;
        #endif
        #ifndef LV_LOG_ERROR
        case LV_LOG_LEVEL_ERROR:
            ESP_LOGE(TAG, "%s", buf);
            break;
        #endif
        #if LV_LOG_LEVEL <= LV_LOG_LEVEL_USER
        #endif
        case LV_LOG_LEVEL_USER:
        #if LV_LOG_LEVEL < LV_LOG_LEVEL_NONE
        case LV_LOG_LEVEL_NONE:
            printf("%s\n", buf);
            break;
        #endif
        default:
            break;
    }
}
#endif


#if CONFIG_LV_USE_CUSTOM_MALLOC

#if CONFIG_LV_ESP_MALLOC_IN_SPIRAM
#define ALLOC_CAPS (MALLOC_CAP_SPIRAM)
#else
#define ALLOC_CAPS MALLOC_CAP_DEFAULT
#endif

void lv_mem_init()
{
}


void *lv_malloc_core(size_t size)
{
    //ESP_LOGI(TAG, "malloc %u", size);
    return heap_caps_malloc(size, ALLOC_CAPS);
}


void lv_free_core(void * p)
{
    //ESP_LOGI(TAG, "free %p", p);
    heap_caps_free(p);
}

void *lv_realloc_core(void * p, size_t new_size)
{
    //ESP_LOGI(TAG, "realloc %u", new_size);
    return heap_caps_realloc(p, new_size, ALLOC_CAPS);
}

void lv_mem_monitor_core(lv_mem_monitor_t * mon_p)
{
    /*Not supported*/
    LV_UNUSED(mon_p);
    return;
}


#endif


/* ------------------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------------------ */

void lv_esp_init()
{
    if (g_initialized) {
        ESP_LOGW(TAG, "lv_init: already initialized");
        return;
    }

    #if LV_USE_LOG
    lv_log_register_print_cb(lv_esp_log);
    #endif

    lv_init();

    lv_tick_set_cb(lv_esp_tick_get);
    lv_delay_set_cb(lv_esp_delay);
}


