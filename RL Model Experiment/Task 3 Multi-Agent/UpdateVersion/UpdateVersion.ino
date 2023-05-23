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
int cmdPrevious = 99;
int cmd2Previous = 99;
                  // forwardPin,backwardPin,leftPin,rightPin,divePin,upPin
int controls[7][6] = {{1,0,1,0,1,0}, // Stop d = 0
                      {0,0,1,0,1,0}, // Forward d = 1
                      {1,1,1,0,1,0}, // Backward d = 2
                      {1,0,0,0,1,0}, // Left d = 3
                      {1,0,1,1,1,0}, // Right d = 4
                      {1,0,1,0,0,0}, // Dive d = 5
                      {1,0,1,0,1,1}, // Up d = 6
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
    cmd = Serial.readStringUntil(' ').toInt();
    cmd2 =  Serial.readStringUntil('\r').toInt();
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
    for(int i=23; i<=33; i+=2){
      digitalWrite(i,controls[1][(i-23)/2]);
    }
  }

  if(cmd2 == 2){ // Forward
    for(int i=22; i<=32; i+=2){
      digitalWrite(i,controls2[1][(i-22)/2]);
    }
  }
//************************************************************************************************************************
//************************************************************************************************************************
//************************************************************************************************************************
//************************************************************************************************************************

  if(cmd == 3){ // Left
    for(int i=23; i<=33; i+=2){
      digitalWrite(i,controls[3][(i-23)/2]);
    }
  }

  if(cmd2 == 3){ // Left
    for(int i=22; i<=32; i+=2){
      digitalWrite(i,controls2[3][(i-22)/2]);
    }
  }
//************************************************************************************************************************
//************************************************************************************************************************
  
  if(cmd == 0){ // Right
    for(int i=23; i<=33; i+=2){
      digitalWrite(i,controls[4][(i-23)/2]);
    }
  }

  if(cmd2 == 0){ // Right

    for(int i=22; i<=32; i+=2){
      digitalWrite(i,controls2[4][(i-22)/2]);
    }
  }
//************************************************************************************************************************
//************************************************************************************************************************
 
  if(cmd == 4){ // Up
    for(int i=23; i<=33; i+=2){
      digitalWrite(i,controls[6][(i-23)/2]);
    } 
  }

  if(cmd == 5){ // Dive
    for(int i=23; i<=33; i+=2){
      digitalWrite(i,controls[5][(i-23)/2]);
    }
  }

  if(cmd2 == 4){ // Up
    for(int i=22; i<=32; i+=2){
      digitalWrite(i,controls2[6][(i-22)/2]);
    }
  }

  if(cmd2 == 5){ // Dive
    for(int i=22; i<=32; i+=2){
      digitalWrite(i,controls2[5][(i-22)/2]);
    } 
  }
//************************************************************************************************************************
//************************************************************************************************************************
 
  if(cmd == 6){ // Keep floating
    int floatInterval1 = 1000;
    int floatInterval2 = 1000;
    if((currentTime - storedFloatTime >= 0) && (currentTime - storedFloatTime < floatInterval1)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[5][(i-23)/2]);
      }
    }
    else if((currentTime - storedFloatTime >= floatInterval1) && (currentTime - storedFloatTime < floatInterval1+floatInterval2)){
      for(int i=23; i<=33; i+=2){
        digitalWrite(i,controls[6][(i-23)/2]);
      }
    }
    else{
      storedFloatTime = currentTime;
    }
  }

  if(cmd2 == 6){ // Keep floating
    int floatInterval1 = 1000;
    int floatInterval2 = 1000;
    if((currentTime - storedFloatTime >= 0) && (currentTime - storedFloatTime < floatInterval1)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[5][(i-22)/2]);
      }
    }
    else if((currentTime - storedFloatTime >= floatInterval1) && (currentTime - storedFloatTime < floatInterval1+floatInterval2)){
      for(int i=22; i<=32; i+=2){
        digitalWrite(i,controls2[6][(i-22)/2]);
      }
    }
    else{
      storedFloatTime = currentTime;
    }
  }
}
