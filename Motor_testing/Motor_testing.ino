#include <Arduino.h>
#include <avr/io.h>

//#define sec 1300000

unsigned long count;
int Motor;

void setup () {
  Serial.begin (9600);
  DDRD = 0b11110000;
  Motor = 0b00000000;
}

void delay_ms (uint16_t millisecond) {
  unsigned long sec;
  sec = (1300000/1000)*millisecond;
  for (count = 0; count < sec; count ++);
}


void loop () {
Motor = 0b10000000;
PORTD = Motor;
Serial.print ("For:  ");
Serial.println (PORTD);
delay(1000);
Motor = Motor + 0b11000000;
PORTD = Motor; 
Serial.print ("Back:  ");
Serial.println (PORTD);
delay(1000);
Motor = Motor + 0b01000000;
PORTD = Motor; 
Serial.print ("Back:  ");
Serial.println (PORTD);
delay(1000);
}

