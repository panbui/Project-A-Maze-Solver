#include <Arduino.h>
#include <avr/io.h>

//#define sec 1300000

unsigned long count;
int For = 0b01100000;
int Back = 0b10010000;
int Release = 0b00000000;
int Left = 0b10100000;
int Right = 0b01010000;

void setup () {
  Serial.begin (9600);
  DDRD = 0b11110000;
}

void setPWM6 (uint8_t PWM6){
  TCCR0A = 0b10000011;
  TCCR0B = 0b00000001;
  OCR0A = PWM6;
}

void setPWM5 (uint8_t PWM5){
  TCCR1A = 0b00100011;
  TCCR1B = 0b00000001;
  OCR1B = PWM5;
}

void delay_ms (uint16_t millisecond) {
  unsigned long sec;
  sec = (1300000/1000)*millisecond;
  for (count = 0; count < sec; count ++);
}

void Forward (){
  PORTD = For;
}

void Backward (){
  PORTD = Back;
}

void Stop (){
  PORTD = Release;
}

void TurnLeft (){
  PORTD = Left;
}

void TurnRight (){
  PORTD = Right;
}


void loop () {
  Forward;
  delay_ms (1000);
  Backward;
  delay_ms(1000);
  Forward;
  delay_ms(1000);
  TurnLeft;
  delay_ms(500);
  TurnRight;
  delay_ms(500);

  int a;
  for (a = 0; a <= 255; a++){
  setPWM6 (a);
  Forward;
  delay_ms (10);
  }

  for (a = 0; a <= 255; a++){
  setPWM5 (a);
  Forward;
  delay_ms (10);
  }

  for (a = 0; a <= 255; a++){
  setPWM5 (a);
  setPWM6 (a);
  Forward;
  delay_ms (10);
  }
}

