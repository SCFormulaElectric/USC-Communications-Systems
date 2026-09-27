#include "Tasks/Status/status_led_task.h"

#include "Peripherals/digital_pins.h"


void status_led_task(void *argument)
{
    //TODO: track elapsed time using xTaskGetTickCount(); //what type does it return?

    (void)argument;
    for (;;) {
        //TODO: Toggle the pin. Use port: AMS_STATUS_LED_GPIO_PORT and pin: AMS_STATUS_LED_PIN
        
        //TODO: delay until you need to blink again... vTaskDelayUntil(); //we need a variable we created
    }
}

task_entry_t create_status_led_task(app_data_t *data)
{
    task_entry_t entry = {0};
    //TODO: call xTaskCreate() to create the task, given STATUS_LED_STACK_SIZE and STATUS_LED_PRIO
    // store it into a variable of the proper type, called "status"


    //TODO: this check must match the variable used to store xTaskCreate() return type
    //configASSERT(status == pdPASS);

    //
    vTaskSuspend(entry.handle);
    entry.name = "status_led";
    return entry;
}
