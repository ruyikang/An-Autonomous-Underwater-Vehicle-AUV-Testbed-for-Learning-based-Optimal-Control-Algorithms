int forwardPin = 23;
int backwardPin = 25;
int leftPin = 27;
int rightPin = 29;
int divePin = 31;
int upPin = 33;
int cmd = 10;
int PIDinp = 0;
int serial_input_flag = 0;

int forwardPin2 = 22;
int backwardPin2 = 24;
int leftPin2 = 26;
int rightPin2 = 28;
int divePin2 = 30;
int upPin2 = 32;
int cmd2 = 10;

unsigned long currentTime,currentTimePID, previousTime;
unsigned long storedTime = 0;
unsigned long storedTime2 = 0;
unsigned long storedFloatTime = 0;

double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
int cmdFlag = 99;
                  // forwardPin,backwardPin,leftPin,rightPin,divePin,upPin
int controls[11][6] = {{1,0,1,0,1,0}, // Stop d = 0
                      {0,0,1,0,1,0}, // Forward d = 1
                      {1,1,1,0,1,0}, // Backward d = 2
                      {1,0,0,0,1,0}, // Left d = 3
                      {1,0,1,1,1,0}, // Right d = 4
                      {1,0,1,0,0,0}, // Dive d = 5
                      {1,0,1,0,1,1}, // Up d = 6
                      {0,0,1,1,1,0}, // Forward + Right d = 7
                      {0,0,0,0,1,0}, // Forward + Left d = 8
                      {1,1,1,1,1,0}, // Back + Right d = 9
                      {1,1,0,0,1,0}  // Back + Left d = 10
                      };
int controls2[7][6] = {{1,0,1,0,1,0}, // Stop d = 0
                      {0,0,1,0,1,0}, // Forward d = 1
                      {1,1,1,0,1,0}, // Backward d = 2
                      {1,0,0,0,1,0}, // Left d = 3
                      {1,0,1,1,1,0}, // Right d = 4
                      {1,0,1,0,0,0}, // Dive d = 5
                      {1,0,1,0,1,1}, // Up d = 6
                      };
                 
int diveFlag = 0;
int upFlag = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  Setup of pins for Submarine 1 (40MHz)
  for(int i = 23; i<=33; i+=2){
    pinMode(i, OUTPUT);
  }
  
  for(int i = 22; i<=32; i+=2){
    pinMode(i, OUTPUT);
  }
  
//  Setup of pins for Submarine 2 (27MHz)
}

void loop() {
  currentTime = millis();
  
  // Receive signal data from Pyton terminal
  if(Serial.available() > 0){
    cmd = Serial.readStringUntil('\r').toInt();
    cmd2 =  10;
    storedTime = millis();
    storedTime2 = millis();
    }

  
  // Basic operating
  if(cmd == 10){ // Stop
    for(int i=23; i<=33; i+=2){
      digitalWrite(i,controls[0][(i-23)/2]);
    }
  }

  if(cmd2 == 10){ // Stop
    for(int i=22; i<=32; i+=2){
      digitalWrite(i,controls2[0][(i-22)/2]);
    }
  }
  
  if(cmd == 2){ // Forward
    int timeDelay1 = 500;
    int timeDelay2 = 0;
    int timeDelay3 = 100;
    int timeDelay4 = 100;

    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[1][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1) && (currentTime - storedTime < timeDelay1+timeDelay2)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[2][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1+timeDelay2)&&(currentTime - storedTime < timeDelay1+timeDelay2+timeDelay3)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[1][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1+timeDelay2+timeDelay3)&&(currentTime - storedTime < timeDelay1+timeDelay2+timeDelay3+timeDelay4)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[2][(i-23)/2]);
      }
    }

    if(currentTime - storedTime >= timeDelay1+timeDelay2+timeDelay3+timeDelay4){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }
  }

  if(cmd2 == 2){ // Forward
    int timeDelay1 = 300;
    int timeDelay2 = 100;
    int timeDelay3 = 100;
    int timeDelay4 = 100;

    if((currentTime - storedTime2 >= 0) && (currentTime - storedTime2 < timeDelay1)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[1][(i-22)/2]);
      }
    }

    if((currentTime - storedTime2 >= timeDelay1) && (currentTime - storedTime2 < timeDelay1+timeDelay2)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[2][(i-22)/2]);
      }
    }

    if((currentTime - storedTime2 >= timeDelay1+timeDelay2)&&(currentTime - storedTime2 < timeDelay1+timeDelay2+timeDelay3)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[1][(i-22)/2]);
      }
    }

    if((currentTime - storedTime2 >= timeDelay1+timeDelay2+timeDelay3)&&(currentTime - storedTime2 < timeDelay1+timeDelay2+timeDelay3+timeDelay4)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[2][(i-22)/2]);
      }
    }

    if(currentTime - storedTime2 >= timeDelay1+timeDelay2+timeDelay3+timeDelay4){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[5][(i-22)/2]);
      }
    }
  }
//************************************************************************************************************************
//************************************************************************************************************************
  if(cmd == 1){ // Backward
    int timeDelay1 = 300;
    int timeDelay2 = 200;

    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[2][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1) && (currentTime - storedTime < timeDelay1+timeDelay2)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[2][(i-23)/2]);
      }
    }

    if(currentTime - storedTime >= timeDelay1+timeDelay2){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }
  }

  if(cmd2 == 1){ // Backward
    int timeDelay1 = 400;
    int timeDelay2 = 200;

    if((currentTime - storedTime2 >= 0) && (currentTime - storedTime2 < timeDelay1)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[2][(i-22)/2]);
      }
    }

    if((currentTime - storedTime2 >= timeDelay1) && (currentTime - storedTime2 < timeDelay1+timeDelay2)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[1][(i-22)/2]);
      }
    }

    if(currentTime - storedTime2 >= timeDelay1+timeDelay2){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[5][(i-22)/2]);
      }
    }
  }
//************************************************************************************************************************
//************************************************************************************************************************

  if(cmd == 3){ // Left
    int timeDelay1 = 200;
    int timeDelay2 = 120;
//    int timeDelay3 = 100;
//    int timeDelay4 = 120;
    int timeDelay3 = 0;
    int timeDelay4 = 0;

    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[3][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1) && (currentTime - storedTime < timeDelay1+timeDelay2)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[4][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1+timeDelay2)&&(currentTime - storedTime < timeDelay1+timeDelay2+timeDelay3)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[3][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1+timeDelay2+timeDelay3)&&(currentTime - storedTime < timeDelay1+timeDelay2+timeDelay3+timeDelay4)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[4][(i-23)/2]);
      }
    }

    if(currentTime - storedTime >= timeDelay1+timeDelay2+timeDelay3+timeDelay4){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }
  }

  if(cmd2 == 3){ // Left
    int timeDelay1_sub2 = 160;
    int timeDelay2_sub2 = 100;
//    int timeDelay3_sub2 = 160;
//    int timeDelay4_sub2 = 120;
    int timeDelay3_sub2 = 0;
    int timeDelay4_sub2 = 0;
    if((currentTime - storedTime2 >= 0) && (currentTime - storedTime2 < timeDelay1_sub2)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[3][(i-22)/2]);
      }
    }

    if((currentTime - storedTime2 >= timeDelay1_sub2) && (currentTime - storedTime2 < timeDelay1_sub2+timeDelay2_sub2)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[4][(i-22)/2]);
      }
    }

    if((currentTime - storedTime2 >= timeDelay1_sub2+timeDelay2_sub2)&&(currentTime - storedTime2 < timeDelay1_sub2+timeDelay2_sub2+timeDelay3_sub2)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[3][(i-22)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1_sub2+timeDelay2_sub2+timeDelay3_sub2)&&(currentTime - storedTime < timeDelay1_sub2+timeDelay2_sub2+timeDelay3_sub2+timeDelay4_sub2)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[4][(i-22)/2]);
      }
    }

    if(currentTime - storedTime2 >= timeDelay1_sub2+timeDelay2_sub2+timeDelay3_sub2+timeDelay4_sub2){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[5][(i-22)/2]);
      }
    }
  }
//************************************************************************************************************************
//************************************************************************************************************************
  
  if(cmd == 0){ // Right
    int timeDelay1 = 220;
    int timeDelay2 = 120;
//    int timeDelay3 = 100;
//    int timeDelay4 = 120;
    int timeDelay3 = 0;
    int timeDelay4 = 0;

    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[4][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1) && (currentTime - storedTime < timeDelay1+timeDelay2)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[3][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1+timeDelay2)&&(currentTime - storedTime < timeDelay1+timeDelay2+timeDelay3)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[4][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1+timeDelay2+timeDelay3)&&(currentTime - storedTime < timeDelay1+timeDelay2+timeDelay3+timeDelay4)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[3][(i-23)/2]);
      }
    }

    if(currentTime - storedTime >= timeDelay1+timeDelay2+timeDelay3+timeDelay4){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }
  }

  if(cmd2 == 0){ // Right
    int timeDelay1 = 180;
    int timeDelay2 = 150;
//    int timeDelay3 = 180;
//    int timeDelay4 = 150;
    int timeDelay3 = 0;
    int timeDelay4 = 0;

    if((currentTime - storedTime2 >= 0) && (currentTime - storedTime2 < timeDelay1)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[4][(i-22)/2]);
      }
    }

    if((currentTime - storedTime2 >= timeDelay1) && (currentTime - storedTime2 < timeDelay1+timeDelay2)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[3][(i-22)/2]);
      }
    }

    if((currentTime - storedTime2 >= timeDelay1+timeDelay2)&&(currentTime - storedTime2 < timeDelay1+timeDelay2+timeDelay3)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[4][(i-22)/2]);
      }
    }

    if((currentTime - storedTime2 >= timeDelay1+timeDelay2+timeDelay3)&&(currentTime - storedTime2 < timeDelay1+timeDelay2+timeDelay3+timeDelay4)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[3][(i-22)/2]);
      }
    }

    if(currentTime - storedTime2 >= timeDelay1+timeDelay2+timeDelay3+timeDelay4){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[5][(i-22)/2]);
      }
    }
  }
//************************************************************************************************************************
//************************************************************************************************************************
 
  if(cmd == 4){ // Up
    int timeDelay1 = 200;
    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[6][(i-23)/2]);
      }
    }

    if(currentTime - storedTime >= timeDelay1){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[0][(i-23)/2]);
      }
    }
  }

  if(cmd == 5){ // Dive
    int timeDelay1 = 800;
    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }

    if(currentTime - storedTime >= timeDelay1){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[0][(i-23)/2]);
      }
    }
  }

  if(cmd2 == 4){ // Up
    int timeDelay1 = 500;
    if((currentTime - storedTime2 >= 0) && (currentTime - storedTime2 < timeDelay1)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[6][(i-22)/2]);
      }
    }

    if(currentTime - storedTime2 >= timeDelay1){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[0][(i-22)/2]);
      }
    }
  }

  if(cmd2 == 5){ // Dive
    int timeDelay1 = 160;
    if((currentTime - storedTime2 >= 0) && (currentTime - storedTime2 < timeDelay1)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[5][(i-22)/2]);
      }
    }

    if(currentTime - storedTime2 >= timeDelay1){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[0][(i-22)/2]);
      }
    }
  }
//************************************************************************************************************************
//************************************************************************************************************************
 
  if(cmd == 6){ // Forward + Right
    int timeDelay1_forward = 250;
    int timeDelay1_turn = 120;
    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1_turn)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[7][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1_turn) && (currentTime - storedTime < timeDelay1_forward)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[1][(i-23)/2]);
      }
    }

    if(currentTime - storedTime >= timeDelay1_forward){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }   
  }

  if(cmd == 7){ // Forward + Left
    int timeDelay1_forward = 250;
    int timeDelay1_turn = 120;
    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1_turn)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[8][(i-23)/2]);
      }
    }

    if((currentTime - storedTime >= timeDelay1_turn) && (currentTime - storedTime < timeDelay1_forward)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[1][(i-23)/2]);
      }
    }

    if(currentTime - storedTime >= timeDelay1_forward){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }
  }


  if(cmd == 8){ // Forward + Right
    int timeDelay1 = 500;
    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[9][(i-23)/2]);
      }
    }

    if(currentTime - storedTime2 >= timeDelay1){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }
  }

  
  if(cmd == 9){ // Forward + Right
    int timeDelay1 = 500;
    if((currentTime - storedTime >= 0) && (currentTime - storedTime < timeDelay1)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[10][(i-23)/2]);
      }
    }

    if(currentTime - storedTime2 >= timeDelay1){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }
  }
}


//double computePID(double inp){
//       
//        elapsedTime = (double)(currentTimePID - previousTime);
//
//        error = setPoint - inp;
//        cumError += error * elapsedTime;
//        rateError = (error - lastError)/elapsedTime;
//
//        double out = kp*error + ki*cumError + kd*rateError;
//
//        lastError = error;
//        previousTime = currentTimePID;
//
//        return out;
//  }
