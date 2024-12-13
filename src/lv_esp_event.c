#include "lv_esp_event.h"

#include <stdio.h>
#include <string.h>
#include <sys/queue.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
#include <esp_log.h>
#include <lvgl.h>

static const char TAG[] = "lv_esp";


#if CONFIG_LV_ESP_EVENT_TASK_CORE_0
#define TIMER_TASK_AFFINITY 0
#elif CONFIG_LV_ESP_EVENT_TASK_CORE_1
#define TIMER_TASK_AFFINITY 0
#else
#define TIMER_TASK_AFFINITY tskNO_AFFINITY
#endif

#if CONFIG_LV_ESP_EVENT_TASK_QUEUE_IN_SPIRAM
#define QUEUE_BUFFER_ATTR EXT_RAM_BSS_ATTR
#else
#define QUEUE_BUFFER_ATTR 
#endif



typedef struct lv_esp_event_handler_node {
    lv_esp_event_handler_t handler;
    void *handler_arg;

    SLIST_ENTRY(lv_esp_event_handler_node) next;
} lv_esp_event_handler_node_t;

typedef SLIST_HEAD(lv_esp_event_handler_nodes, lv_esp_event_handler_node) lv_esp_event_handler_nodes_t;


typedef struct lv_esp_event_id_node {
    int32_t id;                                                     /**< id number of the event */
    lv_esp_event_handler_nodes_t handlers;                             /**< list of handlers to be executed when
                                                                            this event is raised */
    SLIST_ENTRY(lv_esp_event_id_node) next;                            /**< pointer to the next event node on the linked list */
} lv_esp_event_id_node_t;

typedef SLIST_HEAD(lv_esp_event_id_nodes, lv_esp_event_id_node) lv_esp_event_id_nodes_t;


typedef struct lv_esp_event_base_node {
    lv_esp_event_base_t base;

    lv_esp_event_handler_nodes_t handlers;

    lv_esp_event_id_nodes_t id_nodes;

    SLIST_ENTRY(lv_esp_event_base_node) next;
} lv_esp_event_base_node_t;

typedef SLIST_HEAD(lv_esp_event_base_nodes, lv_esp_event_base_node) lv_esp_event_base_nodes_t;


typedef struct {
    TaskHandle_t task;
    RingbufHandle_t queue;
    SemaphoreHandle_t mutex;

    bool notify_pending;
    portMUX_TYPE notify_mux;

    lv_esp_event_base_nodes_t base_nodes;
} lv_esp_evt_drv_t;


static lv_esp_evt_drv_t g_evt_drv;



/* -------------------------------------------------------------------------------------------------------------
 * Event posting
 * ------------------------------------------------------------------------------------------------------------- */

bool lv_esp_event_post_acquire(lv_esp_event_base_t base, lv_esp_event_id_t id, void **event_buf, size_t event_size, TickType_t wait)
{
    lv_esp_evt_drv_t *drv = &g_evt_drv;
    assert(event_size >= sizeof(lv_esp_event_t));

    if (xRingbufferSendAcquire(drv->queue, event_buf, event_size, wait)) {
        lv_esp_event_t *event = *event_buf;
        event->base = base;
        event->id = id;
        return true;
    }
    return false;
}


bool lv_esp_event_post_complete(void *event_buf)
{
    lv_esp_evt_drv_t *drv = &g_evt_drv;
    return xRingbufferSendComplete(drv->queue, event_buf) == pdTRUE;
}


/* -------------------------------------------------------------------------------------------------------------
 * Specific event functions
 * ------------------------------------------------------------------------------------------------------------- */
LV_ESP_EVENT_DEFINE_BASE(_LV_ESP_EVENT_INTERNAL);

enum {
    _LV_ESP_EVENT_NOTIFY,
    _LV_ESP_EVENT_INDEV_READ,
};

typedef struct {
    lv_esp_event_t base;
    lv_indev_t *indev;
} lv_esp_event_indev_t;




bool lv_esp_event_post_notify()
{
    lv_esp_evt_drv_t *drv = &g_evt_drv;
    lv_esp_event_t *event;

    taskENTER_CRITICAL(&drv->notify_mux);
    if (drv->notify_pending) {
        taskEXIT_CRITICAL(&drv->notify_mux);
        return true;
    }
    drv->notify_pending = true;
    taskEXIT_CRITICAL(&drv->notify_mux);

    if (lv_esp_event_post_acquire(_LV_ESP_EVENT_INTERNAL, _LV_ESP_EVENT_NOTIFY, (void**)&event, sizeof(*event), 0)) {
        return lv_esp_event_post_complete(event);
    }

    taskENTER_CRITICAL(&drv->notify_mux);
    drv->notify_pending = false;
    taskEXIT_CRITICAL(&drv->notify_mux);

    return true;
}




bool lv_esp_event_post_indev(lv_indev_t *indev, TickType_t wait)
{
    lv_esp_event_indev_t *event;
    if (lv_esp_event_post_acquire(_LV_ESP_EVENT_INTERNAL, _LV_ESP_EVENT_INDEV_READ, (void**)&event, sizeof(lv_esp_event_indev_t), wait)) {
        event->indev = indev;
        return lv_esp_event_post_complete(event);
    }
    return false;
}






static void display_refr_request_cb(lv_event_t * e)
{
    lv_esp_evt_drv_t *drv = lv_event_get_user_data(e);
    // If the current thread is not the event thread post event to wake it up
    if (xTaskGetCurrentTaskHandle() != drv->task) {
        lv_esp_event_post_notify();
    }
}


static void internal_event_handler(lv_esp_event_t *event, size_t event_size, void *arg)
{
    lv_esp_evt_drv_t *drv = &g_evt_drv;
    switch (event->id) {
        case _LV_ESP_EVENT_NOTIFY:
            {
                taskENTER_CRITICAL(&drv->notify_mux);
                drv->notify_pending = false;
                taskEXIT_CRITICAL(&drv->notify_mux);
            }
            break;
        case _LV_ESP_EVENT_INDEV_READ:
            {
                lv_esp_event_indev_t *evt = (lv_esp_event_indev_t*)event;
                assert(event_size==sizeof(lv_esp_event_indev_t));
                lv_indev_read(evt->indev);
            }
            break;
    }
}


/* -------------------------------------------------------------------------------------------------------------
 * Event dispatching
 * ------------------------------------------------------------------------------------------------------------- */

static void execute_handler(lv_esp_event_handler_node_t *node, lv_esp_event_t *event, size_t event_size)
{
    node->handler(event, event_size, node->handler_arg);
}

static void lv_esp_event_dispatch(lv_esp_evt_drv_t *drv, lv_esp_event_t *event, size_t event_size)
{
    lv_esp_event_handler_node_t *handler, *temp_handler;
    lv_esp_event_base_node_t *base_node, *temp_base;
    lv_esp_event_id_node_t *id_node, *temp_id_node;

    xSemaphoreTakeRecursive(drv->mutex, portMAX_DELAY);

    SLIST_FOREACH_SAFE(base_node, &drv->base_nodes, next, temp_base) {
        if (base_node->base == event->base) {
            SLIST_FOREACH_SAFE(handler, &base_node->handlers, next, temp_handler) {
                execute_handler(handler, event, event_size);
            }

            SLIST_FOREACH_SAFE(id_node, &base_node->id_nodes, next, temp_id_node) {
                if (id_node->id == event->id) {
                    SLIST_FOREACH_SAFE(handler, &id_node->handlers, next, temp_handler) {
                        execute_handler(handler, event, event_size);
                    }
                    break;
                }
            }
        }
    }

    xSemaphoreGiveRecursive(drv->mutex);
}



static void handler_add(lv_esp_event_handler_nodes_t *handlers, lv_esp_event_handler_t handler, void *handler_arg)
{
    lv_esp_event_handler_node_t *node = calloc(1, sizeof(lv_esp_event_handler_node_t));

    node->handler = handler;
    node->handler_arg = handler_arg;

    if (SLIST_EMPTY(handlers)) {
        SLIST_INSERT_HEAD(handlers, node, next);
    }
    else {
        lv_esp_event_handler_node_t *it = NULL;
        lv_esp_event_handler_node_t *last = NULL;

        SLIST_FOREACH(it, handlers, next) {
            last = it;
        }
        SLIST_INSERT_AFTER(last, node, next);
    }
}


static void base_node_add_handler(lv_esp_event_base_node_t *base_node, lv_esp_event_id_t id, lv_esp_event_handler_t handler, void *handler_arg)
{
    if (id==LV_ESP_EVENT_ANY_ID) {
        handler_add(&base_node->handlers, handler, handler_arg);
    }
    else {
        lv_esp_event_id_node_t *it = NULL;
        lv_esp_event_id_node_t *id_node = NULL;
        lv_esp_event_id_node_t *last_id_node = NULL;

        SLIST_FOREACH(it, &(base_node->id_nodes), next) {
            if (it->id == id) {
                id_node = it;
            }
            last_id_node = it;
        }

        if (!id_node) {
            id_node = calloc(1, sizeof(lv_esp_event_id_node_t));
            id_node->id = id;
            SLIST_INIT(&id_node->handlers);

            if (last_id_node) {
                SLIST_INSERT_AFTER(last_id_node, id_node, next);
            }
            else {
                SLIST_INSERT_HEAD(&base_node->id_nodes, id_node, next);
            }
        }
        handler_add(&id_node->handlers, handler, handler_arg);
    }
}

void lv_esp_event_handler_register(lv_esp_event_base_t base, lv_esp_event_id_t id, lv_esp_event_handler_t handler, void *handler_arg)
{
    lv_esp_evt_drv_t *drv = &g_evt_drv;
    lv_esp_event_base_node_t *it = NULL;
    lv_esp_event_base_node_t *base_node = NULL;
    lv_esp_event_base_node_t *last_base_node = NULL;

    xSemaphoreTakeRecursive(drv->mutex, portMAX_DELAY);
    
    SLIST_FOREACH(it, &drv->base_nodes, next) {
        if (it->base == base) {
            base_node = it;
        }
        last_base_node = it;
    }

    if (!base_node) {
        base_node = calloc(1, sizeof(lv_esp_event_base_node_t));
        base_node->base = base;
        SLIST_INIT(&base_node->handlers);
        SLIST_INIT(&base_node->id_nodes);

        if (last_base_node) {
            SLIST_INSERT_AFTER(last_base_node, base_node, next);
        }
        else {
            SLIST_INSERT_HEAD(&drv->base_nodes, base_node, next);
        }
    }
    base_node_add_handler(base_node, id, handler, handler_arg);

    xSemaphoreGiveRecursive(drv->mutex);
}



/* -------------------------------------------------------------------------------------------------------------
 * Event Task
 * ------------------------------------------------------------------------------------------------------------- */


static void lv_esp_event_task(void *params)
{
    lv_esp_evt_drv_t *drv = (lv_esp_evt_drv_t*)params;
    TickType_t timeout = 0;
    uint32_t next_ms = 0;
    bool enabled = true;

    size_t evt_buf_sz;
    void *evt_buf;

    while (true) {
        while ( (evt_buf = xRingbufferReceive(drv->queue, &evt_buf_sz, timeout)) ) {
            lv_lock();
            lv_esp_event_dispatch(drv, (lv_esp_event_t*)evt_buf, evt_buf_sz);
            lv_unlock();

            vRingbufferReturnItem(drv->queue, evt_buf);
            timeout = 0; // Only wait for first item
        }

        next_ms = lv_timer_handler();
        if (enabled && next_ms!=LV_NO_TIMER_READY) {
            timeout = (next_ms > 0) ? pdMS_TO_TICKS(next_ms) : 1;
        }
        else {
            timeout = portMAX_DELAY;
        }
        //ESP_LOGI(TAG, "Wait: %lu ms (%lx) -> %lu", next_ms, next_ms, timeout);
    }

    vTaskDelete(NULL);
}


void lv_esp_event_task_init() 
{
    lv_esp_evt_drv_t *drv = &g_evt_drv;

    assert(drv->task==NULL);

    portMUX_INITIALIZE(&drv->notify_mux);

    SLIST_INIT(&drv->base_nodes);

    static StaticSemaphore_t mutex_buf;
    drv->mutex = xSemaphoreCreateRecursiveMutexStatic(&mutex_buf);

    static uint8_t QUEUE_BUFFER_ATTR buf_data[CONFIG_LV_ESP_EVENT_TASK_QUEUE_SIZE];
    static StaticRingbuffer_t buf;
    drv->queue = xRingbufferCreateStatic(sizeof(buf_data), RINGBUF_TYPE_NOSPLIT, buf_data, &buf);

    ESP_LOGI(TAG, "Start LVGL task");
    static StackType_t task_stack[CONFIG_LV_ESP_EVENT_TASK_STACK_SIZE];
    static StaticTask_t task_buf;
    drv->task = xTaskCreateStaticPinnedToCore(lv_esp_event_task, "lvglEvt", CONFIG_LV_ESP_EVENT_TASK_STACK_SIZE, drv, CONFIG_LV_ESP_EVENT_TASK_PRIORITY, task_stack, &task_buf, TIMER_TASK_AFFINITY);

    lv_esp_event_handler_register(_LV_ESP_EVENT_INTERNAL, LV_ESP_EVENT_ANY_ID, internal_event_handler, drv);

    lv_disp_t *disp = NULL;
    while ( (disp = lv_display_get_next(disp)) ) {
        lv_display_add_event_cb(disp, display_refr_request_cb, LV_EVENT_REFR_REQUEST, drv);
    }
}

