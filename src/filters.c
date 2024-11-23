//
// Created by gl on 09.10.24.
//

#include <string.h>
#include "main.h"
#include "can.h"
#include "filters.h"
#include "gpio.h"
#include "event_groups.h"
#include <libopencm3/stm32/can.h>

extern volatile CruiseState Cruise;
extern EventGroupHandle_t stateChanges;
extern CarStatus Car;

void fixACCbuttons(canMsg *msg)
{
    static uint8_t LIM_Pressed = 0;
    static uint8_t ResPlus_Pressed = 0;
    uint8_t try = 20;
    CruiseState prevState;

    if(((SWMStatus*)msg->Data)->ACC_Lim){
        if(!LIM_Pressed){
            LIM_Pressed = 1;
            ((SWMStatus*)msg->Data)->CC_Lim = 2;
            msg->Delay = 10;
            prevState = Cruise;
            while(memcmp(&Cruise,&prevState,sizeof(CruiseState)) == 0 && try--){
                CAN_Transmit(msg);
                vTaskDelay(5);
            }
            return;
        }
    }else{
        LIM_Pressed = 0;
    }

    if(((SWMStatus*)msg->Data)->ACC_Res_Plus){
        if(!ResPlus_Pressed) {
            ResPlus_Pressed = 1;
            if(Cruise.StandBy && Cruise.SetSpeed){
                ((SWMStatus*)msg->Data)->CC_Res = 1;
            }else {
                ((SWMStatus*)msg->Data)->CC_Set_Plus = 1;
            }
            msg->Delay = 10;
            prevState = Cruise;
            while(memcmp(&Cruise,&prevState,sizeof(CruiseState)) == 0 && try--){
                CAN_Transmit(msg);
                vTaskDelay(5);
            }
            return;
        }
    }else{
        ResPlus_Pressed = 0;
    }
}

void eqPresets(canMsg *msg) {
    static uint8_t eqPreset = 0;

    uint8_t eq = (msg->Data[0]>>4)&0x7;

    uint8_t map[8] = { 0, 0, 0, 0x08,0x20,0x10,0x18,0x28};

    if(eq != 0 && eqPreset != eq){
        msg->Id = MMCAN_ACM_EQ_SET;
        msg->DLC = 8;
        msg->Delay = 10;
        msg->CanPort = CAN2;
        memset(msg->Data,0,8);
        msg->Data[3] = 0xF8;

        uint8_t setEq = map[eq];
        if(setEq){
            msg->Data[1] = setEq;
            CAN_Transmit(msg);
        }
    }
}