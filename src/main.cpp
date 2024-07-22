
#include <avr/interrupt.h>
#include <avr/io.h>
#include "protocol.h"

void init();
void tim1init();
void ioinit();
void usart0init();
void usart0Scan();

int uart0Transmit(uint8_t* data, uint8_t dataSize);

uint8_t RX_Cnt = 0;
uint8_t commandState_Cnt = 0;
uint8_t RX_Buff[RX_BUFF_SIZE] = {0};
commandPtr_Typedef commandState[COMMAND_STATE_SIZE] = {0};

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
      switch (RX_Buff[((commandState[lCommandCnt].start-RX_Buff)+1)%RX_BUFF_SIZE])
      {
      case PROTOCOL_ID_CURRENT_PWM_STATE:
        length = PROTOCOL_LENGTH_CURRENT_PWM_STATE;
        break;
      default:
        break;
      }

      uint8_t msg[20] = {0};
      uint8_t index = commandState[lCommandCnt].start - RX_Buff;
      for(int i = 0; i < length; i++)
      {
        msg[i] = RX_Buff[(index+i)%RX_BUFF_SIZE];
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
  tim1init();
  ioinit();
  usart0init();
}

void ioinit()
{
  DDRB |= 1<<5;
  PORTB = 1<<5;
}

void tim1init()
{
  PRR &= ~(1 << PRTIM1);
  TIMSK1 |= 1 << OCIE1A;
  OCR1AH = 0x3D;
  OCR1AL = 0x09;
  TCCR1A = 0;
  TCCR1B =  (1 << WGM12) | (1 << CS10) | (1 << CS12) ;
}

void usart0init()
{
  UBRR0H = 0;
  UBRR0L = 103;
  UCSR0A = 0;
  UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
  UCSR0C =  (1 << UCSZ00) | (1 << UCSZ01);
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
      case PROTOCOL_ID_CURRENT_PWM_STATE:
        length = PROTOCOL_LENGTH_CURRENT_PWM_STATE;
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