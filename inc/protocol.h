#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTOCOL_BUFFER_LENGTH  64

typedef enum {
    COMMAND_ACQUIRE
} command_t;

typedef struct {
    uint16_t sample_period;      /* In us. */
    uint32_t sample_count;       /* Number of samples to acquire. */
} session_param_t;

session_param_t *Protocol_SessionParams(void);
uint8_t          Protocol_ProcessNewPacket(void);
uint8_t          Protocol_SendU16(uint16_t byte);


#endif
