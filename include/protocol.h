#include <stdint.h>
#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#define RX_BUFF_SIZE            50
#define COMMAND_STATE_SIZE      5

#define PROTOCOL_HEADER (uint8_t)0xBC
#define PROTOCOL_END    (uint8_t)0xDC

#define PROTOCOL_ID_PWM_STATE           (uint8_t)0x70
#define PROTOCOL_LENGTH_PWM_STATE_RX    (uint8_t)3
#define PROTOCOL_LENGTH_PWM_STATE       (uint8_t)5
/* 0xBC 0x70 {PWM1 duty} {PWM2 duty} 0xDC */

//Set PWM duty value (structure same as pwm_state)
#define PROTOCOL_ID_PWM_DUTY_SET        (uint8_t)0x71
#define PROTOCOL_LENGTH_PWM_DUTY_SET_RX (uint8_t)5
#define PROTOCOL_LENGTH_PWM_DUTY_SET    (uint8_t)3

//Save current pwm duty values in EEPROM
#define PROTOCOL_ID_PWM_SAVE            (uint8_t)0x72
#define PROTOCOL_LENGTH_PWM_SAVE_RX     (uint8_t)3
#define PROTOCOL_LENGTH_PWM_SAVE        (uint8_t)3

//Load saved pwm duty values from EEPROM
#define PROTOCOL_ID_PWM_LOAD            (uint8_t)0x73
#define PROTOCOL_LENGTH_PWM_LOAD_RX     (uint8_t)3
#define PROTOCOL_LENGTH_PWM_LOAD        (uint8_t)3

#define PWM1_ADDR   (uint8_t)0
#define PWM2_ADDR   (uint8_t)1

typedef struct
{
    uint8_t* start;
    uint8_t* end;
    int8_t  relevance;
}commandPtr_Typedef;

typedef struct 
{
  uint8_t pwm1_duty;
  uint8_t pwm2_duty;
}PWM_State_Typedef;

#endif