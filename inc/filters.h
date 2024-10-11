//
// Created by gl on 09.10.24.
//

#ifndef OPENCANBOX_FILTERS_H
#define OPENCANBOX_FILTERS_H
#include "main.h"

// ACC - Adaptive cruise buttons
// CC - Standard cruise buttons
typedef struct {
    uint8_t d0;

    uint8_t d1b0:5;
    uint8_t ACC_Lim:1;
    uint8_t ACC_Res_Plus:1;
    uint8_t d1b7:1;

    uint8_t d2;
    uint8_t d3;

    uint8_t ACC_Set_Minus:1; // CC_Set_Minus also
    uint8_t ACC_Dist_Plus:1;
    uint8_t ACC_Dist_Minus:1;
    uint8_t d4b4:5;

    uint8_t d5b0:2;
    uint8_t CC_Cruise:1;
    uint8_t ACC_Cruise:1;
    uint8_t CC_Can:1;
    uint8_t CC_Res:1;
    uint8_t d5b6:1;
    uint8_t CC_Set_Plus:1;

    uint8_t d6b0:5;
    uint8_t CC_Lim:2; // 0x2 - pressed
    uint8_t d6b7:1;

    uint8_t d7;
} SWMStatus; // 030

typedef struct {
    uint8_t d0b0:3;
    uint8_t Cruise_StandBy:1;
    uint8_t Cruise_Mode:3; // 1 - ACC, 3 - LIM, 4 - Transition to standby
    uint8_t d0b7:1;

    uint8_t d1b0:5;
    uint8_t Ignition:2; // 1 - Ignition ON, 3 - Accessory?
    uint8_t d1b7:1;

    uint8_t d2;
    uint8_t d3;
    uint8_t d4;
    uint8_t d5;
    uint8_t d6;
    uint8_t d7;
} PCMStatus; // 0C0

typedef struct {
    uint8_t d0;
    uint8_t d1;
    uint8_t d2;
    uint8_t d3;
    uint8_t d4;
    uint8_t d5;
    uint8_t Speed;
    uint8_t d7;
} CruiseSpeed; // 060

void fixACCbuttons(canMsg *msg);

#endif //OPENCANBOX_FILTERS_H
