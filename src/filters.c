//
// Created by gl on 09.10.24.
//

#include <string.h>
#include "main.h"
#include "can.h"
#include "filters.h"
#include "gpio.h"

extern volatile CruiseState Cruise;

void fixACCbuttons(canMsg *msg)
{
    static uint8_t LIM_Pressed = 0;
    static uint8_t ResPlus_Pressed = 0;
    uint8_t try = 20;

    if(((SWMStatus*)msg->Data)->ACC_Lim){
        if(!LIM_Pressed){
            LIM_Pressed = 1;
            ((SWMStatus *) msg->Data)->CC_Lim = 2;
            msg->Delay = 10;
            CruiseState prevState = Cruise;
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
                ((SWMStatus*) msg->Data)->CC_Set_Plus = 1;
            }
            msg->Delay = 10;
            CruiseState prevState = Cruise;
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