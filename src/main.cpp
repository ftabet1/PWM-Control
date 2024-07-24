
#include <avr/interrupt.h>
#include <avr/io.h>
#include "protocol.h"

void init();
void tim1init();
void ioinit();
void usart0init();
void usart0Scan();
void eepromInit();

uint8_t eeprom_write(uint16_t address, uint8_t data);
uint8_t eeprom_read(uint16_t address, uint8_t* data);

int uart0Transmit(uint8_t* data, uint8_t dataSize);


uint8_t RX_Cnt = 0;
uint8_t commandState_Cnt = 0;
uint8_t RX_Buff[RX_BUFF_SIZE] = {0};
commandPtr_Typedef commandState[COMMAND_STATE_SIZE] = {0};
PWM_State_Typedef pwm_state = {0};

uint8_t usart0_msgSize = 0;
uint8_t usart0_msgCnt = 0;
uint8_t* usart0_msg = nullptr;

int main()
{
  init();
  sei();
  
  while(1)
  {
   usart0Scan();
  }
  return 0;  
}

void usart0Scan()
{
    uint8_t lCommandCnt = 0xFF;
    for(uint8_t i = 0; i < COMMAND_STATE_SIZE; i++)
    {
      if(commandState[i].relevance == 1)
      {
        lCommandCnt = i;
        break;
      }
    }
    if(lCommandCnt != 0xFF)
    {
      uint8_t length = 0;
      uint8_t msg[20] = {0};
      msg[0] = 0xBC;
      uint8_t index = commandState[lCommandCnt].start - RX_Buff;
      switch (RX_Buff[(index+1)%RX_BUFF_SIZE])
      {
      case PROTOCOL_ID_PWM_STATE:
        length = PROTOCOL_LENGTH_PWM_STATE;
        msg[1] = PROTOCOL_ID_PWM_STATE;
        msg[2] = pwm_state.pwm1_duty;
        msg[3] = pwm_state.pwm2_duty;
        msg[4] = 0xDC;
        break;
      case PROTOCOL_ID_PWM_DUTY_SET:
        length = PROTOCOL_LENGTH_PWM_DUTY_SET;
        msg[1] = PROTOCOL_ID_PWM_DUTY_SET;
        pwm_state.pwm1_duty = RX_Buff[(index+2)%RX_BUFF_SIZE];
        pwm_state.pwm2_duty = RX_Buff[(index+3)%RX_BUFF_SIZE];
        OCR1AH = 0;
        OCR1AL = pwm_state.pwm1_duty;
        OCR1BH = 0;
        OCR1BL = pwm_state.pwm2_duty;
        if(pwm_state.pwm1_duty == 0) TCCR1A &= ~(1 << COM1A1);
        else TCCR1A |= 1 << COM1A1;
        if(pwm_state.pwm2_duty == 0) TCCR1A &= ~(1 << COM1B1);
        else TCCR1A |= 1 << COM1B1;

        msg[2] = 0xDC;
        break;
      case PROTOCOL_ID_PWM_SAVE:
        length =  PROTOCOL_LENGTH_PWM_SAVE;
        eeprom_write(PWM1_ADDR, pwm_state.pwm1_duty);
        eeprom_write(PWM2_ADDR, pwm_state.pwm2_duty);
        msg[1] = PROTOCOL_ID_PWM_SAVE;
        msg[2] = PROTOCOL_END;
        break;
      case PROTOCOL_ID_PWM_LOAD:
        length =  PROTOCOL_LENGTH_PWM_LOAD;
        eeprom_read(PWM1_ADDR, &pwm_state.pwm1_duty);
        eeprom_read(PWM2_ADDR, &pwm_state.pwm2_duty);
        OCR1AH = 0;
        OCR1AL = pwm_state.pwm1_duty;
        OCR1BH = 0;
        OCR1BL = pwm_state.pwm2_duty;
        msg[1] = PROTOCOL_ID_PWM_LOAD;
        msg[2] = PROTOCOL_END;
         break;
      default:
        break;
      }
      uart0Transmit(msg, length);
      commandState[lCommandCnt].relevance = 0;
    }
}

int uart0Transmit(uint8_t* data, uint8_t dataSize)
{
  if((UCSR0B & (1 << UDRIE0)) != 0)  return 0;

  usart0_msg = data;
  usart0_msgSize = dataSize;
  UCSR0B |= (1 << UDRIE0);
  return 1;
}


void init()
{
  ioinit();
  usart0init();
  tim1init();
  eepromInit();
}

void ioinit()
{
  DDRB |= (1 << 5) | (1 << 1) | (1 << 2);
  PORTB = 0;
}

void tim1init()
{
  PRR &= ~(1 << PRTIM1);
  TIMSK1 |= 1 << OCIE1A;
  OCR1AH = 0x3D;
  OCR1AL = 0x09;
  TCCR1A =  (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM12) | (1 << CS10);

  eeprom_read(PWM1_ADDR, &pwm_state.pwm1_duty);
  eeprom_read(PWM2_ADDR, &pwm_state.pwm2_duty);
  OCR1AH = 0;
  OCR1AL = pwm_state.pwm1_duty;
  OCR1BH = 0;
  OCR1BL = pwm_state.pwm2_duty;
}



void usart0init()
{
  UBRR0H = 0;
  UBRR0L = 103;
  UCSR0A = 0;
  UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
  UCSR0C =  (1 << UCSZ00) | (1 << UCSZ01);
}

void eepromInit()
{
  EECR = 0;
  EEDR = 0;
  EEARH = 0;
  EEARL = 0;
}

uint8_t eeprom_write(uint16_t address, uint8_t data)
{
  cli();
  if((address & 0xFE00) != 0) return 0;
  while((EECR & (1 << EEPE)) != 0);
  EEARH = (uint8_t)(address >> 8);
  EEARL = (uint8_t)(address);
  EEDR = data;
  EECR |= (1 << EEMPE);
  EECR |= (1 << EEPE);
  while(EECR & (1 << EEPE) != 0);
  sei();
  return 1;
}

uint8_t eeprom_read(uint16_t address, uint8_t* data)
{
  cli();
  while(EECR & (1 << EEPE) != 0);
  EEARH = (uint8_t)(address >> 8);
  EEARL = (uint8_t)(address);
  EECR |= (1 << EERE); 
  (*data) = EEDR;
  sei();
  return (*data);
}

ISR(TIMER1_COMPA_vect)
{
  PORTB ^= 1<<5;
}

uint8_t start_listening = 0;
uint8_t local_cnt = 0;
ISR(USART_RX_vect)
{
  RX_Buff[RX_Cnt] = UDR0;
  if(RX_Buff[RX_Cnt] == PROTOCOL_HEADER && start_listening == 0)
  {
    rx_header: 
    start_listening = 1;
    local_cnt = 1;
    if(commandState[commandState_Cnt].relevance != 0)
    {
      for(int i = 0; i < COMMAND_STATE_SIZE; i++)
      {
        if(commandState[(commandState_Cnt+i)%COMMAND_STATE_SIZE].relevance == 0)
        {
          commandState_Cnt = (commandState_Cnt+i)%COMMAND_STATE_SIZE;
          break;
        }
      }
    }
    commandState[commandState_Cnt].start = &RX_Buff[RX_Cnt];
  }
  else if(start_listening == 1)
  {
    local_cnt++;
    uint8_t length = 0;
    if(local_cnt>1)
    {
      switch (RX_Buff[((commandState[commandState_Cnt].start-RX_Buff)+1)%RX_BUFF_SIZE])
      {
      case PROTOCOL_ID_PWM_STATE:
        length = PROTOCOL_LENGTH_PWM_STATE_RX;
        break;
      case PROTOCOL_ID_PWM_DUTY_SET:
        length = PROTOCOL_LENGTH_PWM_DUTY_SET_RX;
        break;
      case PROTOCOL_ID_PWM_SAVE:
        length = PROTOCOL_LENGTH_PWM_SAVE_RX;
      case PROTOCOL_ID_PWM_LOAD:
        length = PROTOCOL_LENGTH_PWM_LOAD_RX;
        break;
      default:
        break;
      }
      if(length == local_cnt && RX_Buff[RX_Cnt] == PROTOCOL_END)
      {
        commandState[commandState_Cnt].end = RX_Buff+RX_Cnt;
        commandState[commandState_Cnt].relevance = 1;
        start_listening = 0;
        local_cnt = 0; 
      }
      else if(length == local_cnt && RX_Buff[RX_Cnt] == PROTOCOL_HEADER)
      {
        goto rx_header;
      }
      else if(length == 0)
      {
        start_listening = 0;
        local_cnt = 0; 
      }
      else if(local_cnt > length)
      {
        if(RX_Buff[RX_Cnt == PROTOCOL_HEADER]) goto rx_header;
        else
        {
          start_listening = 0;
          local_cnt = 0; 
        }
      }
    }
  }
  RX_Cnt = (RX_Cnt+1)%RX_BUFF_SIZE;
}
 
ISR(USART_UDRE_vect)
{
  if(usart0_msgCnt != usart0_msgSize)
  {
    UDR0 = usart0_msg[usart0_msgCnt++];
  }
  else
  {
    usart0_msg = nullptr;
    usart0_msgCnt = 0;
    usart0_msgSize = 0;
    UCSR0B &= ~(1 << UDRIE0);
  }
}