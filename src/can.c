//
// Created by gl on 24.05.24.
//

#include "main.h"
#include "can.h"
#include "gpio.h"
#include "event_groups.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>

QueueHandle_t canTxQueue;
QueueHandle_t canRxQueue;

void can_tx_task(void *arg);
void can_rx_task(void *arg);

void can_setup()
{
    canTxQueue = xQueueCreate( 5, sizeof(canMsg) );
    canRxQueue = xQueueCreate( 5, sizeof(canMsg) );

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
    AFIO_MAPR |= AFIO_MAPR_CAN2_REMAP;

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

    /* CAN2 cell init. */
    if (can_init(CAN2,
                 false,           /* TTCM: Time triggered comm mode? */
                 true,            /* ABOM: Automatic bus-off management? */
                 true,           /* AWUM: Automatic wakeup mode? */
                 false,           /* NART: No automatic retransmission? */
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
            (HSCAN_BCM_SWM << 5),
            (HSCAN_BCM_SWM << 5),
            (CAN_DIAG_ID << 5),
            0,
            true);

    /* CAN filter 0 init. */
    can_filter_id_list_16bit_init(
            14,
            (HSCAN_BCM_SWM << 5),
            (HSCAN_BCM_SWM << 5),
            (HSCAN_BCM_SWM << 5),
            (CAN_DIAG_ID << 5),
            0,
            true);

    /* Enable CAN RX interrupt. */
    can_enable_irq(CAN1, CAN_IER_FMPIE0);
    can_enable_irq(CAN2, CAN_IER_FMPIE0);

    xTaskCreate(can_tx_task, "canTx", 128, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(can_rx_task, "canRx", 128, NULL, configMAX_PRIORITIES-2, NULL);
}

static void can_rx_isr(uint32_t canport)
{
    BaseType_t xTaskWokenByReceive;
    bool ext, rtr;
    uint8_t fmi;
    canMsg msg;

    can_receive(canport, 0, true, (uint32_t*)&msg.Id, &ext, &rtr, &fmi, &msg.DLC, msg.Data, NULL);

    msg.CanPort = canport;
    xQueueSendFromISR(canRxQueue,&msg,&xTaskWokenByReceive);
    portYIELD_FROM_ISR(xTaskWokenByReceive);

    can_fifo_release(canport, 0);
}

void usb_lp_can_rx0_isr(void)
{
    can_rx_isr(CAN1);
}

void can2_rx0_isr(void)
{
    can_rx_isr(CAN2);
}

void can_rx_task(void *arg)
{
    canMsg msg;

    for(;;)
    {
        if( xQueueReceive( canRxQueue,&msg,portMAX_DELAY ) == pdPASS )
        {
            switch (msg.Id){
                case 0x707:
                    LED1_TOGGLE();
                    msg.Id = 0x70F;
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