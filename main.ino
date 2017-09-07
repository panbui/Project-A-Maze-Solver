  // VARIABLES FOR MAZE SOLVING AND PATH OPTIMISATION
bool finish = false;
bool memorised = false;
char option[50];
int index = 0;

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
