#ifndef STATUS_LED_TASK_H
#define STATUS_LED_TASK_H

#include "app.h"

#define STATUS_LED_STACK_SIZE        (KILOBYTE / 2U)
/* 1 Hz heartbeat: toggle every half period. */
#define STATUS_LED_HALF_PERIOD_MS    500U

void status_led_task(void *argument);
task_entry_t create_status_led_task(app_data_t *data);

#endif /* STATUS_LED_TASK_H */
