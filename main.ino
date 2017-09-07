#include <avr/io.h>

unsigned long count;
uint8_t sensor[5];
char Case;

float activeSensor = 0; // Count active sensors
float totalSensor = 0; // Total sensor readings
float avgSensor = 1.5; // Average sensor reading

float Kp = 66;
float Ki = 0.05;
float Kd = 2;

float error = 0;
float *er_pt = &error;      // error pointer
float previousError = 0;
float totalError = 0;
int PIDstatus = 1;

float power = 0;

  // VARIABLES FOR MAZE SOLVING AND PATH OPTIMISATION
bool finish = false;
bool memorised = false;
char option[50];
int index = 0;

void delay_ms (uint16_t millisecond) {
  unsigned long sec;
  sec = ((16000000/12)/1000)*millisecond;
  for (count = 0; count < sec; count ++);
}

void setPWM_leftmotor (uint8_t PWM6){
  OCR0A = PWM6;
}

void setPWM_rightmotor (uint8_t PWM5){
  OCR0B = PWM5;
}
 
void Forward (){
PORTB = 0b00001010;
}

void Backward (){
 PORTB = 0b00000101;
}

void Stop (){
  //PORTB = 0b00000000;
  setPWM_leftmotor(0);
  setPWM_rightmotor(0);
}

void TurnLeft (){
PORTB = 0b00000110;
}

void TurnRight (){
 PORTB = 0b00001001;
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
    if ((sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) 
    || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==0)
    || (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0)
    || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1)
    || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1)
    || (sensor[0]==0 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)
    || (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0)
    || (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)
    || (sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)) {
    
    
      Case = 'S'; // Straight 00100
      //Serial.println ("Straight");
    }

// TURN RIGHT
    else if ((sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1)
          || (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1)) {
      Case = 'R'; // Turn Right 00111
                  //            01111
      //Serial.println ("Right");
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
}

void AddOpt(char opt) {
  option[index] = opt;
  Optimise();
  index ++;
}

void Optimise() {
  if (index >= 2 && option[index - 1] == 'U') {
    switch (option[index - 2]) {
      case 'L':
        // L-U-L => S
        if (option[index] == 'L') {
          option[index - 2] = 'S';
        }
        // L-U-S => R
        else if (option[index] == 'S') {
          option[index - 2] = 'R';
        }
        break;

      case 'S':
        // S-U-L => R
        if (option[index] == 'L') {
          option[index - 2] = 'R';
        }
        break;

      case 'R':
        // R-U-L => U
        if (option[index] == 'L') {
          option[index - 2] = 'U';
        }
        break;
    }
    index -= 2;
  }
}

void RunCase (){
   SensorsCondition ();
   switch (Case){
      
      case 'S': // Straight
        Forward ();
        if (PIDstatus ==1)
        PID_program();
        Serial.println ("Straight");
        break;
        
      case 'R': // Right
          PIDstatus =0;
          // STEP UP
          PORTD ^= (1<< PORTD3);
  
          ReadSensors();
          // TURN RIGHT
          if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) {
            setPWM_leftmotor (200);
            setPWM_rightmotor (200);
            TurnRight();
            delay_ms(140);
            Stop();
            PIDstatus=1;
            // ADD TO OPTION ARRAY
            AddOpt('R');
            Serial.println ("Turn Right");
          }
          // GO STRAIGHT
          else {
            setPWM_leftmotor (200);
            setPWM_rightmotor (200);
            Forward ();
            PIDstatus=1;
            // ADD TO OPTION ARRAY
            AddOpt('S');
            Serial.println("Straight00");
          }
        break;
        
      case 'L': // Left 
        // TURN LEFT
        PIDstatus=0;
        PORTD ^= (1<< PORTD3);
        setPWM_leftmotor (200);
        setPWM_rightmotor (200);
        TurnLeft ();
        delay_ms(100);
        Stop();
        PIDstatus=1;
        // ADD TO OPTION ARRAY
        AddOpt('L');
        Serial.println ("TurnLeft");
        break;
        
      case 'D': // Deadend 
      //PIDstatus=0;
        setPWM_leftmotor (190);
        setPWM_rightmotor (190);
        TurnRight ();
        delay_ms(100);
        PIDstatus=1;
        Serial.println ("DeadEnd");
        // ADD TO OPTION ARRAY
        AddOpt('U');
        break;
        
      case 'F': // All black
        PIDstatus=0;
        // Move 1 step forward
        PORTD ^= (1<< PORTD3);

        ReadSensors();
        // FINISH
        if (sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1) {
          Stop();
          // ADD TO OPTION ARRAY
          AddOpt('F');
          Serial.println ("Finish");
        }
        // TURN LEFT
        else {
          //while(~(PINC & (1<<3))){
          setPWM_leftmotor (200);
          setPWM_rightmotor (200);
          TurnLeft ();
          //}
          delay_ms(120);
          Stop();
          PIDstatus=1;
          // ADD TO OPTION ARRAY
          AddOpt('L');
          Serial.println("Turn Left");
        }
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
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000001;
  DDRD = 0b01100000;
  DDRB = 0b00001111;
  DDRC = 0b00000000;
  PORTD = 0b01100000;
  DDRD &= ~(1<<DDD3);
  EICRA |= (1<<ISC10);
  EIMSK |= (1<<INT1);
  sei();
}

void SecondRun() {
  ReadSensors();
  // CHOOSE FROM OPTION ARRAY WHEN ENCOUNTERING INTERSECTION
  if (!((sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) 
     || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==0)
     || (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0)
     || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1)
     || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1)
     || (sensor[0]==0 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)
     || (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0)
     || (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)
     || (sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0))) { // or whatever conditions that indicates intersection
    switch (option[index]) {
      case 'F':
        Stop();
        break;

      case 'L':
        // TURN LEFT
        break;

      case 'R':
        // TURN RIGHT
        break;

      case 'S':
        // GO STRAIGHT
        break;
    }
    // GO TO NEXT OPTION
    index ++;
  }
}

void loop() {
  // FIRST RUN - EXPLORE THE MAZE
  if (finish == false) {
    RunCase ();
  }
  // FIND THE OPTIMAL PATH
  else if (memorised == false) {
    index = 0;
    delay_ms(5000);
    // MOVE OUT OF FINISH BOX
    setPWM_leftmotor (200);
    setPWM_rightmotor (200);
    Forward();
    delay_ms(700);
    Stop ();
    delay_ms(700);
    memorised = true;  
  }
  // SECOND RUN - OPTIMAL PATH
  else {
    SecondRun();
  }
}

void PID_program()
{ 
    Error(er_pt);
    
    previousError = error; // save previous error for differential 
 
    totalError += error; // Accumulate error for integral
    
    power = (Kp*error) + (Kd*(error-previousError)) + (Ki*totalError);
    
    if( power>255 ) { power = 230; }
    if( power<-255 ) { power = -230; }
    
    if(power<0) // Turn left
    {
      setPWM_rightmotor(190);
      setPWM_leftmotor(195 - abs(int(power)));
    }
    
    else // Turn right
    {
      setPWM_rightmotor(195 - int(power));
      setPWM_leftmotor(190);
    }
   
}

ISR(INT1_vect){
  setPWM_leftmotor (200);
  setPWM_rightmotor (200);
  Forward();
  delay_ms(150);
  Stop ();  
}

void Error(float *error) {
  for(int i=0; i<=4; i++) 
    {
      if(sensor[i]==1) {
        activeSensor+=1; 
        }
      totalSensor += sensor[i] * (i+1);
    }
      
    avgSensor = totalSensor/activeSensor;
    *error = (avgSensor - 3);
    activeSensor = 0; totalSensor = 0;
}
