/*
 * Logician Firmware
 * USB Logic Analyzer for STM32F4 Discovery
 * Written by Dan Bridges 2013
 * dbridges @ github
 */

#include "discovery.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#include "protocol.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

#define DAQ_PORT                GPIOD
#define DAQ_BUFFER_LEN          50000
#define DAQ_TRIGGER_TIMEOUT     10000000
#define DAQ_TRIGGER_OFFSET      200  // Number of samples before
                                    // trigger to keep
typedef enum {
    TRIGGER_RISING,
    TRIGGER_FALLING
} trigger_t;

extern uint8_t protocol_rx_buffer[64];

volatile uint32_t sample_count;
volatile uint8_t aq_byte;
volatile uint8_t aq_flag = 1;
volatile uint32_t daq_i = 0;
volatile uint32_t start_i = 0;
volatile uint8_t daq_buffer[DAQ_BUFFER_LEN];
volatile uint8_t triggered = FALSE;
volatile uint8_t trigger_type;
volatile uint8_t trigger_channel;
volatile uint8_t prev_sample;
volatile uint32_t timeout;


static void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(DAQ_PORT, &GPIO_InitStructure);
}

static void usb_cdc_init(void)
{
    USBD_Init(&USB_OTG_dev,
              USB_OTG_FS_CORE_ID,
              &USR_desc,
              &USBD_CDC_cb,
              &USR_cb);
}

static void TIM2_Config(void)
{
    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // TIM2 Periph clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Time base configuration
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 1;
    TIM_TimeBaseStructure.TIM_Period = 83;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // TIM2 TRGO selection
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
}

void TIM2_Enable(void)
{
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_Disable(void)
{
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM2, DISABLE);
}


void TIM2_IRQHandler(void)
{
    if (triggered) {
        // Acquisiton has already been triggered, just sample and save.
        if (sample_count > 0) {
            // Each sample is 4 bits so we alternate storage between the 4 MSBs
            // and 4 LSBs. We then save the 2 data samples as a single byte to
            // an array.
            if (aq_flag) {
                aq_byte = ((uint8_t)DAQ_PORT->IDR) << 4;
            } else {
                aq_byte |= ((uint8_t)DAQ_PORT->IDR) & 0x0F;
                daq_buffer[daq_i] = aq_byte;
                daq_i++;
                if (daq_i > DAQ_BUFFER_LEN)
                    daq_i = 0;
            }
            aq_flag = !aq_flag;
            sample_count--;
        }
    } else {
        // Still waiting for acquisition to be triggered, look for correct
        // change in signal at trigger channel.
        uint8_t sample;
        sample = ((uint8_t)DAQ_PORT->IDR);
        if (aq_flag) {
            aq_byte = sample << 4;
        } else {
            aq_byte |= sample & 0x0F;
            daq_buffer[daq_i] = aq_byte;
            daq_i++;
            if (daq_i > DAQ_BUFFER_LEN)
                daq_i = 0;
        }
        aq_flag = !aq_flag;
        switch (trigger_type) {
            case TRIGGER_RISING:
                // Look for 0 -> 1 transition.
                if (((prev_sample >> trigger_channel) & 0x01) == 0 &&
                        ((sample >> trigger_channel) & 0x01) == 1)
                    triggered = TRUE;
                break;
            case TRIGGER_FALLING:
                // Look for 1 -> 0 transition.
                if (((prev_sample >> trigger_channel) & 0x01) == 1 &&
                        ((sample >> trigger_channel) & 0x01) == 0)
                    triggered = TRUE;
                break;
        }
        prev_sample = sample;
        if (triggered) {
            // Trigger was just satisified, store the index to the array where
            // it occurred for use later when transfering data.
            LED_Reset(LED_ALL);
            LED_Set(LED_RED);
            if (daq_i > DAQ_TRIGGER_OFFSET)
                start_i = daq_i - DAQ_TRIGGER_OFFSET;
            else {
                start_i = DAQ_BUFFER_LEN - (DAQ_TRIGGER_OFFSET - daq_i);
            }
        } else {
            timeout--;
            if (timeout == 0) {
                sample_count = 0;
            }
        }
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void reset_acquistion(void)
{
    daq_i = 0;
    start_i = 0;
    triggered = FALSE;
    aq_flag = 1;
    timeout = DAQ_TRIGGER_TIMEOUT;
    LED_Reset(LED_ALL);
    LED_Set(LED_GREEN);
}

uint8_t check_usb(void)
{
    if (VCP_data_available() >= 64) {
        VCP_get_buffer(protocol_rx_buffer, 64);
        Protocol_ProcessNewPacket();
        return 1;
    }
    return 0;
}

int main(void)
{
    uint8_t data_byte;
    unsigned int start, current;
    session_param_t *params;

    gpio_init();
    LED_Init();
    usb_cdc_init();
    VCP_flush_rx();
    TIM2_Config();
    reset_acquistion();

    while (1) {
        while(!check_usb());  // Wait for usb command.
        params = Protocol_SessionParams();
        switch (params->command) {
            case COMMAND_ACQUIRE:
                sample_count = params->sample_count -
                    (DAQ_TRIGGER_OFFSET);
                trigger_type = params->trigger_type;
                trigger_channel = params->trigger_channel;
                prev_sample = (uint8_t)DAQ_PORT->IDR;
                LED_Reset(LED_ALL);
                LED_Set(LED_ORANGE);
                TIM2_Enable();
                while (sample_count > 0);  // Wait for trigger and acquisition.
                TIM2_Disable();
                LED_Reset(LED_ALL);
                if ((DAQ_BUFFER_LEN - start_i) > params->sample_count / 2) {
                    VCP_send_buffer((uint8_t *)&daq_buffer[start_i],
                            daq_i - start_i);
                } else {
                    LED_Set(LED_BLUE);
                    VCP_send_buffer((uint8_t *)&daq_buffer[start_i],
                            DAQ_BUFFER_LEN - start_i);
                    VCP_send_buffer((uint8_t *)daq_buffer,
                            (params->sample_count / 2) -
                                (DAQ_BUFFER_LEN - start_i));
                }
                break;
            case COMMAND_INFO:
                VCP_send_str("Version 0.1a\n");
        }
        reset_acquistion();
    }

    return 0;
}
