#include "protocol.h"
#include "usbd_cdc_vcp.h"

uint8_t protocol_rx_buffer[PROTOCOL_BUFFER_LENGTH];

static  session_param_t session_params;

uint8_t    Protocol_ProcessNewPacket(void)
{
    uint8_t *ptr;

    ptr = &protocol_rx_buffer[1];
    
    switch (protocol_rx_buffer[0]) {
        case COMMAND_ACQUIRE:
            session_params.sample_period = (*(uint16_t*)ptr);
            ptr += 2;
            session_params.sample_count = (*(uint32_t*)ptr);
            break;
        default:
            return 0;
    }
    return 1;
}

session_param_t *Protocol_SessionParams(void)
{
    return &session_params;
}

uint8_t Protocol_SendU16(uint16_t val)
{
    VCP_put_char((uint8_t)(val & 0x00FF));
    VCP_put_char((uint8_t)(val >> 8));
    return 2;
}
