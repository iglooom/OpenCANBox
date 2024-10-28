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

typedef struct {
    uint8_t SetSpeed;
    uint8_t StandBy:1;
    uint8_t LimMode:1;
    uint8_t CruiseMode:1;
} CruiseState;

typedef struct {
    uint8_t Ignition:1;
    uint8_t DesiredGear;
    uint8_t ActualGear;
    uint8_t ShifterPosition;
    int16_t BatteryCurrent;
    uint8_t BatteryCharge;
    uint16_t BatteryVoltage;
    int16_t CoolantTemp;
} CarStatus;

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
    uint8_t Ignition:1; // 1 - Ignition ON, 3 - Accessory?
    uint8_t d1b7:2;

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

typedef struct {
    uint8_t d0;

    uint8_t d1b0:4;
    uint8_t ActualGear:4; // E - reverse

    uint8_t DesiredGear:4; // E - reverse
    uint8_t d2b4:4;

    uint8_t d3;
    uint8_t d4;
    uint8_t d5;
    uint8_t d6;
    uint8_t d7;
} PcmGear; // 0D0

typedef struct {
    uint8_t d0;
    uint8_t d1;
    uint8_t d2;
    uint8_t d3;


    uint8_t d4b0:3;
    uint8_t ShifterPos:3; // 0-P, 1-R, 2-N, 3-D, 4-S

    uint8_t d5;
    uint8_t d6;
    uint8_t d7;
} PcmShifter; // 0E0

typedef struct {
    uint8_t CurrentH:7;
    uint8_t d0b7:1;
    uint8_t d1b0:4;
    uint8_t CurrentL:4;

    uint8_t d2;
    uint8_t d3;
    uint8_t d4;
    uint8_t d5;
    uint8_t d6;
    uint8_t d7;
} BatCurrent;

typedef struct {
    uint8_t d0;
    uint8_t d1;
    uint8_t d2;
    uint8_t d3;
    uint8_t d4;
    uint8_t CoolantTemp; // value +60
    uint8_t d6;
    uint8_t d7;
} EngineTemp; // 2F0

typedef struct {
    uint8_t d0;
    uint8_t d1;
    uint8_t d2;
    uint8_t d3;
    uint8_t d4;

    uint8_t volts_div_16;

    uint8_t d6;
    uint8_t SoC:7;
    uint8_t d7b0:1;
} BattVoltage; // 435

#define CAN_DIAG_ID 0x707

#define HSCAN_BCM_SWM 0x030
#define HSCAN_PCM_SPD 0x060
#define HSCAN_PCM_STATUS 0x0C0
#define HSCAN_PCM_GEAR 0x0D0
#define HSCAN_PCM_SHIFTER 0x0E0
#define HSCAN_PCM_TEMP 0x2F0
#define HSCAN_BMS 0x150
#define HSCAN_BATT_VOLT 0x435

#define MMCAN_NAV_APIM 0x2C0
#define MMCAN_NAV_IPC_FC 0x2C8

#define MMCAN_RDS_ACM 0x2B4
#define MMCAN_RDS_APIM_FC 0x2BC

#define APIM_FC (1 << 0)
#define FC_Received (1 << 1)

void isotp_send(uint16_t txId, uint16_t fcId, uint8_t* data, uint16_t len);
void isotp_fc_cb(canMsg *msg);

void can_setup(void);
void CAN_Transmit(canMsg *msg);

#endif //CAN_H
