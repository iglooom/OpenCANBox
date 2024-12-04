//
// Created by gl on 26.10.24.
//
#include <math.h>
#include <memory.h>
#include "can.h"
#include "stdio.h"

extern CarStatus Car;

void ipc_print_task(void *arg __attribute((unused)))
{
    char txt[30];
    uint8_t text[41] = { 0 };
    text[0] = 0x20;
    uint8_t disp[] = { 0x22, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };

    for (;;) {
        //if(Car.Ignition && !Car.NavInfoPresent) {
            memset(txt,0,sizeof(txt));
            // only 16 chars fit
            sprintf(txt, "%d\xB0 %d%% %dA %d.%dV", Car.CoolantTemp, Car.BatteryCharge, Car.BatteryCurrent, Car.BatteryVoltage / 1000, Car.BatteryVoltage%1000/100);

            // Dirty hack convert to Unicode zero page. Each char in txt turned into 0x00 0xXX in text.
            char *p = txt;
            uint8_t text_cntr = 4;
            while(*p){
                text[text_cntr] = *p;
                p++;
                text_cntr+=2;
            }

            isotp_send(MMCAN_NAV_APIM, 0, text, sizeof(text));
            vTaskDelay(50);
            isotp_send(MMCAN_NAV_APIM, 0, disp, sizeof(disp));
        //}
        vTaskDelay(500);
    }
}

void ipc_print_init()
{
    xTaskCreate(ipc_print_task, "ipc_print", 256, NULL, configMAX_PRIORITIES-3, NULL);
}


