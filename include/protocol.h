#include <stdint.h>
#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#define RX_BUFF_SIZE            50
#define COMMAND_STATE_SIZE      5

#define PROTOCOL_HEADER (uint8_t)0xBC
#define PROTOCOL_END    (uint8_t)0xDC

#define PROTOCOL_ID_CURRENT_PWM_STATE       (uint8_t)0x70
#define PROTOCOL_LENGTH_CURRENT_PWM_STATE   (uint8_t)5
/* 0xBC 0x70 {PWM1 duty} {PWM2 duty} 0xDC */

typedef struct
{
    uint8_t* start;
    uint8_t* end;
    int8_t  relevance;
}commandPtr_Typedef;

#endif