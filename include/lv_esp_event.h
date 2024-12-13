
#pragma once

#include <freertos/FreeRTOS.h>
#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LV_ESP_EVENT_DECLARE_BASE(id) extern lv_esp_event_base_t const id
#define LV_ESP_EVENT_DEFINE_BASE(id) lv_esp_event_base_t const id = #id

typedef const char *lv_esp_event_base_t;
typedef int32_t lv_esp_event_id_t;

#define LV_ESP_EVENT_ANY_ID (-1)


typedef struct {
    lv_esp_event_base_t base;
    lv_esp_event_id_t id;
} lv_esp_event_t;


typedef void (*lv_esp_event_handler_t)(lv_esp_event_t *event, size_t event_size, void *arg);

bool lv_esp_event_post_acquire(lv_esp_event_base_t base, lv_esp_event_id_t id, void **event_buf, size_t event_size, TickType_t wait);
bool lv_esp_event_post_complete(void *event_buf);


void lv_esp_event_handler_register(lv_esp_event_base_t base, lv_esp_event_id_t id, lv_esp_event_handler_t handler, void *handler_arg);


bool lv_esp_event_post_notify();
bool lv_esp_event_post_indev(lv_indev_t *indev, TickType_t wait);


void lv_esp_event_task_init();

#ifdef __cplusplus
}
#endif
