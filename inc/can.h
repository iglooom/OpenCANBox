//
// Created by gl on 24.05.24.
//

#ifndef CAN_H
#define CAN_H
#include "main.h"

typedef struct {
    uint32_t CanPort;
    uint16_t Id;
    uint8_t DLC;
    uint8_t Data[8];
    uint8_t Delay;
} canMsg;

#define CAN_DIAG_ID 0x707

#define HSCAN_BCM_SWM 0x030
#define HSCAN_PCM_SPD 0x060
#define HSCAN_PCM_STATUS 0x0C0

void can_setup(void);
void CAN_Transmit(canMsg *msg);

#endif //CAN_H
