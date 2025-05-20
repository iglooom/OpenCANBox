//
// Created by gl on 24.05.24.
//

#include "main.h"
#include "can.h"
#include "gpio.h"
#include "event_groups.h"
#include "filters.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include "string.h"
#include <libopencm3/stm32/iwdg.h>
#include <libopencmsis/core_cm3.h>
#include "ipc_print.h"


QueueHandle_t canTxQueue;
QueueHandle_t canRxQueue;

EventGroupHandle_t isotpEvGrp;
volatile uint8_t BlockSize, SeparationTime;
volatile uint16_t WaitFcIde;

TimerHandle_t   navInfoTimeout;
//TimerHandle_t canSleepTimer;

void can_tx_task(void *arg);
void can_rx_task(void *arg);

volatile CruiseState Cruise;
volatile CarStatus Car = {0};

void onCanIdle()
{
    CAN_Drv_Slp();
}

void onNavInfoTimeout()
{
    Car.NavInfoPresent = 0;
}

void can_setup()
{
    memset(&Cruise,0,sizeof(Cruise));
    canTxQueue = xQueueCreate( 5, sizeof(canMsg) );
    canRxQueue = xQueueCreate( 5, sizeof(canMsg) );
    isotpEvGrp = xEventGroupCreate();
//    canSleepTimer = xTimerCreate("canSleep",10000,pdFALSE,( void * ) 0,onCanIdle);
    navInfoTimeout = xTimerCreate("navInfoTimeout",3000,pdFALSE,( void * ) 0,onNavInfoTimeout);

    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_CAN1);
    rcc_periph_clock_enable(RCC_CAN2);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    // CAN1
    /* Configure CAN1 pin: RX (input pull-up). */
    gpio_set_mode(GPIO_BANK_CAN1_RX, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_RX);
    gpio_set(GPIO_BANK_CAN1_RX, GPIO_CAN1_RX);

    /* Configure CAN1 pin: TX. */
    gpio_set_mode(GPIO_BANK_CAN1_TX, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_TX);

    /* NVIC setup. */
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
    nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);

    // CAN2
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ, AFIO_MAPR_CAN2_REMAP);

    /* Configure CAN2 pin: RX (input pull-up). */
    gpio_set_mode(GPIO_BANK_CAN2_RE_RX, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN2_RE_RX);
    gpio_set(GPIO_BANK_CAN2_RE_RX, GPIO_CAN2_RE_RX);

    /* Configure CAN2 pin: TX. */
    gpio_set_mode(GPIO_BANK_CAN2_RE_TX, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN2_RE_TX);

    /* NVIC setup. */
    nvic_enable_irq(NVIC_CAN2_RX0_IRQ);
    nvic_set_priority(NVIC_CAN2_RX0_IRQ, 2);

    /* Reset CAN. */
    can_reset(CAN1);
    can_reset(CAN2);

    /* CAN1 cell init. 500 kbit/s */
    if (can_init(CAN1,
                 false,           /* TTCM: Time triggered comm mode? */
                 true,            /* ABOM: Automatic bus-off management? */
                 true,           /* AWUM: Automatic wakeup mode? */
                 false,           /* NART: No automatic retransmission? */
                 false,           /* RFLM: Receive FIFO locked mode? */
                 true,           /* TXFP: Transmit FIFO priority? */
                 CAN_BTR_SJW_1TQ,
                 CAN_BTR_TS1_8TQ,
                 CAN_BTR_TS2_7TQ,
                 4,
                 false,
                 false))             /* BRP+1: Baud rate prescaler */
    {
        /* Die because we failed to initialize. */
        while (1)
            __asm__("nop");
    }

    /* CAN2 cell init. */
    if (can_init(CAN2,
                 false,           /* TTCM: Time triggered comm mode? */
                 true,            /* ABOM: Automatic bus-off management? */
                 true,           /* AWUM: Automatic wakeup mode? */
                 true,           /* NART: No automatic retransmission? */
                 false,           /* RFLM: Receive FIFO locked mode? */
                 true,           /* TXFP: Transmit FIFO priority? */
                 CAN_BTR_SJW_1TQ,
                 CAN_BTR_TS1_13TQ,
                 CAN_BTR_TS2_2TQ,
                 4,
                 false,
                 false))             /* BRP+1: Baud rate prescaler */
    {
        /* Die because we failed to initialize. */
        while (1)
            __asm__("nop");
    }

    /* CAN filter 0 init. */
    can_filter_id_list_16bit_init(
            0,
            (HSCAN_BCM_SWM << 5),
            (HSCAN_PCM_SPD << 5),
            (HSCAN_PCM_STATUS << 5),
            (CAN_DIAG_ID << 5),
            0,
            true);

    can_filter_id_list_16bit_init(
            1,
            (HSCAN_PCM_GEAR << 5),
            (HSCAN_PCM_SHIFTER << 5),
            (HSCAN_BMS << 5),
            (HSCAN_PCM_GEAR << 5),
            0,
            true);

    can_filter_id_list_16bit_init(
            2,
            (HSCAN_PCM_TEMP << 5),
            (HSCAN_BATT_VOLT << 5),
            (HSCAN_PCM_TEMP << 5),
            (HSCAN_PCM_TEMP << 5),
            0,
            true);

    /* CAN filter 0 init. */
    can_filter_id_list_16bit_init(
            14,
            (MMCAN_NAV_IPC_FC << 5),
            (MMCAN_APIM_LIGHT << 5),
            (MMCAN_ACM_EQ_SET << 5),
            (MMCAN_NAV_APIM << 5),
            0,
            true);

    /* Enable CAN RX interrupt. */
    can_enable_irq(CAN1, CAN_IER_FMPIE0);
    can_enable_irq(CAN2, CAN_IER_FMPIE0);

    xTaskCreate(can_tx_task, "canTx", 128, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(can_rx_task, "canRx", 128, NULL, configMAX_PRIORITIES-2, NULL);

    CAN_Drv_EN();
}

static void can_rx_isr(uint32_t canport)
{
    BaseType_t xTaskWokenByReceive;
    bool ext, rtr;
    uint8_t fmi;
    canMsg msg;

    can_receive(canport, 0, true, (uint32_t*)&msg.Id, &ext, &rtr, &fmi, &msg.DLC, msg.Data, NULL);

    msg.CanPort = canport;
    switch (msg.Id) {
        case HSCAN_PCM_SPD:
            Cruise.SetSpeed = ((CruiseSpeed*)msg.Data)->Speed;
            break;
        case HSCAN_PCM_STATUS:
            Cruise.StandBy = ((PCMStatus*)msg.Data)->Cruise_StandBy;
            Car.Ignition = (((PCMStatus*)msg.Data)->Ignition&1);
            switch (((PCMStatus*)msg.Data)->Cruise_Mode) {
                case 1:
                    Cruise.CruiseMode = 1;
                    Cruise.LimMode = 0;
                    break;
                case 3:
                    Cruise.LimMode = 1;
                    Cruise.CruiseMode = 0;
                    break;
                case 4:
                    break;
                default:
                    Cruise.LimMode = 0;
                    Cruise.CruiseMode = 0;
            }
            break;
        case HSCAN_PCM_GEAR:
            Car.ActualGear = ((PcmGear*)msg.Data)->ActualGear;
            Car.DesiredGear = ((PcmGear*)msg.Data)->DesiredGear;
            break;
        case HSCAN_PCM_SHIFTER:
            Car.ShifterPosition = ((PcmShifter*)msg.Data)->ShifterPos;
            if(Car.ShifterPosition == 1){
                SW_ON();
            }else{
                SW_OFF();
            }
            break;
        case HSCAN_BMS:
            Car.BatteryCurrent = (int16_t)(((uint16_t)(((BatCurrent*)msg.Data)->CurrentH)<<4) + ((BatCurrent*)msg.Data)->CurrentL - 512);
            break;
        case HSCAN_PCM_TEMP:
            Car.CoolantTemp = ((int16_t)((EngineTemp *)msg.Data)->CoolantTemp) - 60;
            break;
        case HSCAN_BATT_VOLT:
            uint16_t mv = ((BattVoltage *)msg.Data)->volts_div_16;
            Car.BatteryVoltage = (mv*62) + (mv>>1);
            Car.BatteryCharge = ((BattVoltage*)msg.Data)->SoC;
            break;
        default:
            xQueueSendFromISR(canRxQueue,&msg,&xTaskWokenByReceive);
            portYIELD_FROM_ISR(xTaskWokenByReceive);
    }

    can_fifo_release(canport, 0);
}

void usb_lp_can_rx0_isr(void)
{
    can_rx_isr(CAN1);
}

void can2_rx0_isr(void)
{
    can_rx_isr(CAN2);
//
//    bool ext, rtr;
//    uint8_t fmi;
//    canMsg msg;
//
//    can_receive(CAN2, 0, true, (uint32_t*)&msg.Id, &ext, &rtr, &fmi, &msg.DLC, msg.Data, NULL);
//    isotp_fc_cb(&msg);
}

void can_rx_task(void *arg)
{
    canMsg msg;

    for(;;)
    {
        if( xQueueReceive( canRxQueue,&msg,portMAX_DELAY ) == pdPASS )
        {
            switch (msg.Id){
                case HSCAN_BCM_SWM:
                    fixACCbuttons(&msg);
                    break;
                case MMCAN_APIM_LIGHT:
                    eqPresets(&msg);
                    break;
                case MMCAN_NAV_APIM:
                    Car.NavInfoPresent = 1;
                    xTimerReset(navInfoTimeout,100);
                    break;
                case CAN_DIAG_ID:
                    switch (msg.Data[1]) {
                        case 0x11: // ECUReset
                            if(msg.Data[2] == 0x01){
                                iwdg_start();
                                msg.Data[0] = 0x02;
                                msg.Data[1] = 0x51;
                                msg.Data[2] = 0x01;
                            }else {
                                msg.Data[0] = 0x02;
                                msg.Data[1] = 0x7F;
                                msg.Data[2] = 0x12; // subFunctionNotSupported
                            }
                            break;
                        case 0x3E: // TesterPresent
                            msg.Data[0] = 0x02;
                            msg.Data[1] = 0x7E;
                            msg.Data[2] = 0x00;
                            ipc_print_stop();
                            break;
                        default:
                            msg.Data[0] = 0x02;
                            msg.Data[1] = 0x7F;
                            msg.Data[2] = 0x11; // serviceNotSupported
                            break;
                    }
                    memset(&msg.Data[3],0,5);
                    msg.Id = CAN_DIAG_RESP_ID;
                    msg.DLC = 8;
                    xQueueSend(canTxQueue,&msg,10);
                    break;
            }
        }
    }
}

void can_tx_task(void *arg)
{
    canMsg msg;

    for(;;){
        xQueueReceive(canTxQueue,&msg,portMAX_DELAY);

        while(can_transmit(msg.CanPort,msg.Id,false,false,msg.DLC,msg.Data) == -1){
            vTaskDelay(10);
        }
        vTaskDelay(msg.Delay);
    }
}

void CAN_Transmit(canMsg *msg)
{
    xQueueSend(canTxQueue,msg,50);
}

void isotp_send(uint16_t txId, uint16_t fcId, uint8_t* data, uint16_t len)
{
    canMsg msg;
    uint8_t sendCntr, CfCntr;

    msg.CanPort = CAN2;
    msg.Id = txId;
    msg.DLC = 8;
    msg.Data[0] = (1<<4) + ((len>>8)&0xF);
    msg.Data[1] = (len & 0xFF);
    memcpy(&msg.Data[2],data,6);
    sendCntr = 6;

    WaitFcIde = fcId;
    SeparationTime = 20;
    xEventGroupClearBits(isotpEvGrp,FC_Received);
    xQueueSend(canTxQueue,&msg,300);

    if( WaitFcIde == 0 || (xEventGroupWaitBits(isotpEvGrp,FC_Received,pdTRUE,pdFALSE,300) & FC_Received)){
        msg.Delay = SeparationTime;
        CfCntr = 1;
        while(sendCntr < len){
            msg.Data[0] = (2<<4) + (CfCntr&0xF);
            CfCntr++;
            if(len - sendCntr >= 7){
                memcpy(&msg.Data[1],&data[sendCntr],7);
                sendCntr+=7;
            }else{
                memset(&msg.Data[1],0x0,7);
                uint8_t last = len - sendCntr;
                memcpy(&msg.Data[1],&data[sendCntr],last);
                sendCntr+=last;
            }
            xQueueSend(canTxQueue,&msg,300);
        }
    }
}

void isotp_fc_cb(canMsg *msg)
{
    if (msg->Data[0] == 0x30 && msg->Id == WaitFcIde) { // FlowControl ContinueToSend
        BlockSize = msg->Data[1];
        SeparationTime = msg->Data[2]&0x7F;
        xEventGroupSetBits(isotpEvGrp, FC_Received);
    }
}