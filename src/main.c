#include <libopencm3/stm32/rcc.h>

#include "main.h"
#include "can.h"
#include "gpio.h"


#ifndef NDEBUG
extern void initialise_monitor_handles(void);
#endif

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName ){
    ( void ) xTask; ( void ) pcTaskName;
    while (1);
}

static void blink_led(void *arg __attribute((unused))) {
    for (;;) {
        LED1_TOGGLE();
        vTaskDelay(1000);
    }
}

int main() {
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSE25_72MHZ]);
#ifndef NDEBUG
    initialise_monitor_handles();
#endif

    gpio_setup();
    can_setup();
    //xTaskCreate(blink_led, "blink", 128, NULL, configMAX_PRIORITIES-3, NULL);

    vTaskStartScheduler();
}
