//
// Created by gl on 24.05.24.
//

#ifndef GSMCAN_CAN_H
#define GSMCAN_CAN_H

typedef struct {
    uint32_t CanPort;
    uint16_t Id;
    uint8_t DLC;
    uint8_t Data[8];
    uint8_t Delay;
} canMsg;

#define CAN_DIAG_ID 0x707

#define HSCAN_BCM_SWM 0x303

void can_setup(void);

#endif //GSMCAN_CAN_H
