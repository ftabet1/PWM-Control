#define __AVR_ATmega328P__
#include <avr/interrupt.h>
#include <avr/io.h>


void init();
void tim1init();
void ioinit();
void usart0init();

int uart0Transmit(uint8_t* data, uint8_t dataSize);
uint8_t usart0_msgSize = 0;
uint8_t usart0_msgCnt = 0;
uint8_t* usart0_msg = nullptr;

int main()
{
  init();
  sei();
  uint8_t data[4] = {0x01, 0x23, 0x45, 0x67};
   uart0Transmit(data, 4);
  while(1)
  {
   
  }
  return 0;  
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