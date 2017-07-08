#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <util/delay.h>
//DDRB = 0b111111111; // Data Direction Register 1 for Output,0 for input
int a;
int main(void){
  while(1){
    TCCR1A = 0b10100011;
    TCCR1B = 0b00000001;
    for (a = 0; a <=255; a++){
    OCR1A = a;
    
    DDRB = 0xFF;
    PORTB = 0xFF;
    _delay_ms(10);
    PORTB = 0x00;
    _delay_ms(10);
    }
  }
};
