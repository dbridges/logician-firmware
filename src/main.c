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

#define DAQ_PORT            GPIOD
#define DAQ_BUFFER_LEN      2000 

extern uint8_t protocol_rx_buffer[64];

static uint8_t daq_buffer[DAQ_BUFFER_LEN];
static uint8_t sampling = FALSE;

volatile unsigned int *DWT_CYCCNT  = (volatile unsigned int *)0xE0001004;
volatile unsigned int *DWT_CONTROL = (volatile unsigned int *)0xE0001000;
volatile unsigned int *SCB_DEMCR   = (volatile unsigned int *)0xE000EDFC;

void enable_timing(void)
{
    static int enabled = 0;
 
    if (!enabled)
    {
        *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
        *DWT_CYCCNT = 0; // reset the counter
        *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
 
        enabled = 1;
    }
}

void timing_delay(unsigned int tick)
{
    unsigned int start, current;

    start = *DWT_CYCCNT;
    do {
        current = *DWT_CYCCNT;
    } while((current - start) < tick);
}

static void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
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

uint8_t check_usb()
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
    uint32_t sample_count = 0;
    session_param_t *params;
    uint32_t daq_i = 0;

    gpio_init();
    usb_cdc_init();
    VCP_flush_rx();
    enable_timing();

    while (1) {
        while(!check_usb());
        /*params = Protocol_SessionParams();*/
        /*sample_count = params->sample_count;*/
        sample_count = 2000;

        while (sample_count > 0) {
            start = *DWT_CYCCNT;
            data_byte = ((uint8_t)DAQ_PORT->IDR) << 4;
            timing_delay(144 - (*DWT_CYCCNT - start) - 1);
            data_byte |= ((uint8_t)DAQ_PORT->IDR) & 0x0F;

            daq_buffer[daq_i] = data_byte;
            daq_i++;
            if (daq_i > DAQ_BUFFER_LEN)
                daq_i = 0;

            sample_count--;
            timing_delay(288 - (*DWT_CYCCNT - start) - 1);
        }

        /* Just finished acquisition. */
        sampling = FALSE;
        VCP_send_buffer(&daq_buffer[daq_i], DAQ_BUFFER_LEN - daq_i);
        VCP_send_buffer(daq_buffer, daq_i);
        daq_i = 0;
    }

    return 0;
}
