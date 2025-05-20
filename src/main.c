#include <libopencm3/stm32/rcc.h>

#include "main.h"
#include "can.h"
#include "gpio.h"
#include "ipc_print.h"
#include "libopencmsis/core_cm3.h"


#ifndef NDEBUG
extern void initialise_monitor_handles(void);
#endif

extern volatile CarStatus Car;

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName ){
    ( void ) xTask; ( void ) pcTaskName;
    while (1);
}

static void blink_led(void *arg __attribute((unused))) {
    for (;;) {
        if(Car.AudioIsOn || Car.Ignition){
            SW2_ON();
        }else{
            SW2_OFF();
        }
//        if(Car.ShifterPosition == 1){
//            LED1_ON();
//        }else{
//            LED1_OFF();
//        }
        LED1_TOGGLE();
        vTaskDelay(500);
    }
}

int main() {
#ifdef BOOTLOADER
    __disable_irq();
    SCB->VTOR = 0x8001800;
    __enable_irq();
#endif
#ifndef NDEBUG
    initialise_monitor_handles();
#endif
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSE25_72MHZ]);

    gpio_setup();
    can_setup();
    ipc_print_init();
    xTaskCreate(blink_led, "blink", 128, NULL, configMAX_PRIORITIES-3, NULL);

    vTaskStartScheduler();
}
