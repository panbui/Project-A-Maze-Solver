  // VARIABLES FOR MAZE SOLVING AND PATH OPTIMISATION
bool finish = false;
bool memorised = false;
char option[50];
int index = 0;

  // CỨ MỖI LẦN ADD VÀO ARRAY SẼ CHẠY OPTIMISE
void AddOpt(char opt) {
  option[index] = opt;
  Optimise();
  index ++;
}

void Optimise() {
  // CHỈ CHẠY OPTIMISE KHI ARRAY CÓ ÍT NHẤT 3 ELEMENT
  // VÀ ELEMENT KẾ CUỐI LÀ U TURN
  if (index >= 2 && option[index - 1] == 'U') {
    // TỚI ĐÂY THÌ XÉT ELEMENT LIỀN TRƯỚC U TURN
    switch (option[index - 2]) {
      case 'L':
        // L-U-L => S _ TRÁI -> QUAY ĐẦU -> TRÁI THÌ THÀNH THẲNG
        if (option[index] == 'L') {
          option[index - 2] = 'S';
        }
        // L-U-S => R _ TRÁI -> QUAY ĐẦU -> THẲNG THÌ THÀNH QUẸO PHẢI
        else if (option[index] == 'S') {
          option[index - 2] = 'R';
        }
        break;

      case 'S':
        // S-U-L => R _ THẲNG -> QUAY ĐẦU -> TRÁI THÌ THÀNH QUẸO PHẢI
        if (option[index] == 'L') {
          option[index - 2] = 'R';
        }
        break;

      case 'R':
        // R-U-L => U _ PHẢI -> QUAY ĐẦU -> TRÁI THÌ THÀNH U TURN
        if (option[index] == 'L') {
          option[index - 2] = 'U';
        }
        break;
    }
    // TỪ 3 ELEMENT RÚT XUỐNG CÒN 1 NÊN INDEX TRỪ ĐI 2 ĐỂ TIẾP TỤC
    index -= 2;
    // VỪA XEM HÌNH VỪA DÒ THEO CÁI ALGORITHM NÀY SẼ HIỂU :)
  }
}

// CỨ MỖI LẦN GẶP NGÃ RẼ SẼ APPEND VÔ ARRAY PATH OPTION TRÊN
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
  // NẾU ĐIỀU KIỆN KHÁC THÌ ĐỔI LẠI, NÓI CHUNG TỚI NGÃ RẼ THÌ SẼ TRIGGER NÓ CHẠY
  if (!((sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) 
     || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==0)
     || (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0)
     || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1)
     || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1)
     || (sensor[0]==0 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)
     || (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0)
     || (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)
     || (sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0))) {
    switch (option[index]) {
      // GẶP F THÌ DỪNG HẲN LUÔN VÌ F NÓ Ở CUỐI CÙNG
      case 'F':
        Stop();
        break;
      
      // GẶP L THÌ QUẸO TRÁI
      case 'L':
        // TURN LEFT
        break;
      
      // GẶP R THÌ QUẸO PHẢI
      case 'R':
        // TURN RIGHT
        break;

      // GẶP S THÌ ĐI THẲNG
      case 'S':
        // GO STRAIGHT
        break;
    }
    // GO TO NEXT OPTION - CỘNG LÊN ĐỂ NÓ QUA ELEMENT KẾ TIẾP
    index ++;
  }
}

void loop() {
  // FIRST RUN - EXPLORE THE MAZE
  if (finish == false) { // BOOLEAN finish ĐƯỢC TRIGGLE true TRONG TRƯỜNG HỢP FINISH TRONG RUNCASE()
    RunCase ();
  }
  // FIND THE OPTIMAL PATH
  else if (memorised == false) {  // CÓ CÁI FLAG memorised ĐỂ MẤY DÒNG DƯỚI CHỈ CHẠY 1 LẦN
    index = 0;
    delay_ms(5000);
    // MOVE OUT OF FINISH BOX - CHO NÓ CHẠY RA KHỎI CÁI BOX FINISH ĐÃ
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
