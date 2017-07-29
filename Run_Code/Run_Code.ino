#include <Arduino.h>
#include <avr/io.h>

unsigned long count;
uint8_t sensor[5];
char Case;


void delay_ms (uint16_t millisecond) {
  unsigned long sec;
  sec = ((16000000/12)/1000)*millisecond;
  for (count = 0; count < sec; count ++);
}

void setPWM_leftmotor (uint8_t PWM6){
  TCCR0A = 0b10000011;
  TCCR0B = 0b00000001;
  OCR0A = PWM6;
}

void setPWM_rightmotor (uint8_t PWM5){
  TCCR0A = 0b00100011;
  TCCR0B = 0b00000001;
  OCR0B = PWM5;
}
 
void Forward (){
  PORTB = 0b00001010;
}

void Backward (){
  PORTB = 0b00000101;
}

void Stop (){
  PORTB = 0b00000000;
}

void TurnLeft (){
  PORTB = 0b00000110;
}

void TurnRight (){
  PORTB = 0b00001001;
}

void TestCodeMotor (){
  // BASIC MOVING FUNCTION AND PWM TESTING
  Forward ();
  Serial.println ("Forward");
  delay_ms (5000);
  
  Backward ();
  Serial.println ("Backward");
  delay_ms (1000);
  
  TurnLeft ();
  Serial.println ("Left");
  delay_ms (300);

  Forward ();
  Serial.println ("Forward");
  delay_ms (5000);
  
  TurnRight ();
  Serial.println ("Right");
  delay_ms (300);

  Forward ();
  Serial.println ("Forward");
  delay_ms (5000);
  
  Stop ();
  Serial.println ("Stop");
  delay_ms (1000);

  for (int a = 0; a <= 255; a+=5){
    setPWM_leftmotor (a);
    Forward ();
    Serial.println ("PWM Left");
    delay_ms (100);
  }

  for (int a = 0; a <= 255; a+=5){
    setPWM_rightmotor (a);
    Backward ();
    Serial.println ("PWM Right");
    delay_ms (100);
  }
}

void ReadSensors (){
  // Sensor 0
    if (PINC & (1<<0)){
      sensor[0] = 1;
    }else{
      sensor[0] = 0;}

// Sensor 1
    if (PINC & (1<<1)){
      sensor[1] = 1;
    }else{
      sensor[1] = 0;}

// Sensor 2
    if (PINC & (1<<2)){
      sensor[2] = 1;
    }else{
      sensor[2] = 0;}

// Sensor 3
    if (PINC & (1<<3)){
      sensor[3] = 1;
    }else{
      sensor[3] = 0;}


// Sensor 4
    if (PINC & (1<<4)){
      sensor[4] = 1;
    }else{
      sensor[4] = 0;}

     //Print in the serial monitor 
    Serial.print (sensor[0]);
    Serial.print ("   ");
    Serial.print (sensor[1]);
    Serial.print ("   ");
    Serial.print (sensor[2]);
    Serial.print ("   ");
    Serial.print (sensor[3]);
    Serial.print ("   ");
    Serial.print (sensor[4]);
    Serial.print ("   ");
}

void SensorsCondition (){
    ReadSensors ();
  // STRAIGHT
    if (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) {
      Case = 'S'; // Straight 00100
      //Serial.println ("Straight");
    }
// CALIBRATE RIGHT
    else if ((sensor[0]==0 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)
          || (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0)
          || (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)
          || (sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)) {
      Case = 'r'; 
      //Serial.println("Calibrate right");
    }

// TURN RIGHT
    else if ((sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1)
          || (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1)) {
      Case = 'R'; // Turn Right 00111
                  //            01111
      //Serial.println ("Right");
    }

// CALIBRATE LEFT
    else if ((sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==0)
          || (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0)
          || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1)
          || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1)) {
      Case = 'l';
      //Serial.println("Calibrate left");
    }

// TURN LEFT
    else if ((sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0)
          || (sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0)) {
      Case = 'L'; // Turn Left 11100
                  //           11110
      //Serial.println ("Left");
    }

// DEADEND
    else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0){
      Case = 'D'; // Dead End 00000
      //Serial.println ("DeadEnd");
    }

// FINISH
    else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1){
      Case = 'F'; // Finish 11111
      //Serial.println ("Finish");
    }

// OTHERS
    else{
       Case = 'O';
       //Serial.println ("Errors");
    }
}

void RunCase (){
   SensorsCondition ();
   switch (Case){
      
      case 'S': // Straight
        setPWM_leftmotor (255);
        setPWM_rightmotor (255);
        Forward ();
        Serial.println ("Straight");
        break;
        
      case 'R': // Right
        setPWM_leftmotor (255);
        setPWM_rightmotor (255);
        TurnRight ();
        Serial.println ("TurnRight");
        break;
        
      case 'r': // Cali right
        Serial.println ("Cali right");
        for (int a = 255; a>=120; a-=5){
          setPWM_rightmotor (a);
          Forward ();
        }
        break;
        
      case 'L': // Left 
        setPWM_leftmotor (255);
        setPWM_rightmotor (255);
        TurnLeft ();
        Serial.println ("TurnLeft");
        break;

      case 'l': // Cali left 
        Serial.println ("Cali left");
        for (int a = 255; a>=120; a-=5){
          setPWM_leftmotor (a);
          Forward ();
        }
        break;
        
      case 'D': // Deadend 
        setPWM_leftmotor (255);
        setPWM_rightmotor (255);
        TurnRight ();
        Serial.println ("DeadEnd");
        break;
        
      case 'F': // Finish
        Stop ();
        Serial.println ("Finish");
        break;
        
      default:
        Stop ();
        Serial.println ("Errors");
        break;
    }
}

void setup() {
//  DDRD |= (1<<5) | (1<<6);
  Serial.begin (115200);
  DDRD = 0b01100000;
  DDRB = 0b00001111;
  DDRC = 0b00000000;
  PORTD = 0b01100000;
}

void loop() {
    //TestCodeMotor ();
     RunCase ();  
}



