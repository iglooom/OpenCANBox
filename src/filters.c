//
// Created by gl on 09.10.24.
//

#include "main.h"
#include "can.h"
#include "filters.h"
#include "gpio.h"

uint8_t ACC_LimitSpeed = 0;
uint8_t ACC_StandByMode = 0;
uint8_t ACC_LimMode = 0;
uint8_t ACC_CruiseMode = 0;

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
            uint8_t currStdByMode = ACC_StandByMode;
            uint8_t currLimMode = ACC_LimMode;
            while(ACC_LimMode == currLimMode && ACC_StandByMode == currStdByMode && try--){
                CAN_Transmit(msg);
            }
            return;
        }
    }else{
        LIM_Pressed = 0;
    }

    if(((SWMStatus*)msg->Data)->ACC_Res_Plus){
        if(!ResPlus_Pressed) {
            ResPlus_Pressed = 1;
            ((SWMStatus*)msg->Data)->CC_Set_Plus = 1;
            msg->Delay = 10;
            uint8_t currSpeed = ACC_LimitSpeed;
            while(currSpeed == ACC_LimitSpeed && try--){
                CAN_Transmit(msg);
            }
            return;
        }
    }else{
        ResPlus_Pressed = 0;
    }
}