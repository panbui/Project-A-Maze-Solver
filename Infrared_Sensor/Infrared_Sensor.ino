#include <Arduino.h>
#include <avr/io.h>
int sensor[2]={0, 0};

void setup() {

Serial.begin (9600);
DDRB = 0b00000000;

}

void loop() {

//Serial.print ("sensor1: ");
if (PINB & (1<<PINB5)){
//  Serial.println ("1");
  sensor[0] = 1;
}else{
//  Serial.println ("0");
    sensor[0] = 0;
}

//Serial.print ("sensor2: ");
if (PINB & (1<<PINB4)){
//  Serial.println ("1");
    sensor[1] = 1;
}else{
//  Serial.println ("0");
    sensor[1] = 0;
}

Serial.print (sensor[0]);
Serial.print ("   ");
Serial.println (sensor[1]);
if (sensor[0] == 1 && sensor[1] == 1){
  Serial.println ("Yes");
}else{
  Serial.println ("No");
}

delay (500);

  
}
