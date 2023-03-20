int forwardPin = 23;
int backwardPin = 25;
int leftPin = 27;
int rightPin = 29;
int upPin = 31;
int divePin = 33;
int cmdFlag = 99;
String cmdOrigin;
String message;
int cmd;
int cmdTime = 100;
int commaposition,blankposition;
int downNum = 5;
int upNum = 5;

unsigned long ms_from_start = 0;
unsigned long ms_previous_read_1;
unsigned long ms_previous_read_2;
unsigned long ms_previous_read_3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200);
  pinMode(forwardPin, OUTPUT);
  pinMode(backwardPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(upPin, OUTPUT);
  pinMode(divePin, OUTPUT);
  
}

void loop() {
//  Forward, Down, Left active low, and using PNP with 10k resistor
//  Backward, Up, Right active High, and using NPN with 1k resistor
  //Cmd: 0->Right 1->Back 2->Forward 3->Left 4->Up 5->Down 6->Fowrward+Right 
//       7->Forward+Left 8->Back+Right 9->Back+Left 10->Stop 
  digitalWrite(forwardPin, HIGH);
  digitalWrite(backwardPin, LOW);
  digitalWrite(leftPin, HIGH);
  digitalWrite(rightPin, LOW);
  digitalWrite(upPin, LOW);
  digitalWrite(divePin, HIGH); 
  while(Serial.available() > 0){
//  cmd = Serial.readStringUntil('\r').;
//  int index = str.indexOf(' ');
//  int cmd1 = str.substring(0, index).toInt();
//  int cmd2 = str.substring(index + 1, str.length()).toInt();
//  Serial1.write(cmd2);
  
  
  cmd =  Serial.readStringUntil('\r').toInt();
  switch (cmd){
    case 10: // Stop
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH); 
      cmdFlag = cmd;
      break;
    case 2: // Forward
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(200);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, HIGH);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(100);
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(100);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, HIGH);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(260);
             
      // Serial.print("cmd =" + cmd);
      // Serial.print("cmdFlag =" + cmdFlag);
      break;
    case 1: //Backward
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, HIGH);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(300);
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(200);
      break;
    case 3: // Left
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(130);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, HIGH);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(140);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(100);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, HIGH);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(120);
      break;
    case 0: // Right
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, HIGH);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(160); // Limitation is 100ms and cannot be less150
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(140); //160
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, HIGH);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(100); // Limitation is 100ms and cannot be less150
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(140); //160
      break;
    case 4: // Up
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, HIGH);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(200);
      break;
    case 5: // Down
//    When the down button keeps working for a period of 1080ms, the bump reaches its limit
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, LOW);
      delay(200);
      break;
    case 7:
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd; 
      delay(cmdTime); 
      break;
    case 6:
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, HIGH);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(cmdTime);
    break;
    case 8:
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, HIGH);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, HIGH);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
    cmdFlag = cmd;
    delay(cmdTime);
    break;
    case 9:
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, HIGH);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
    cmdFlag = cmd;
    delay(cmdTime);
    break;
  }
 }




  

  //String teststr = Serial2.readString();  //read until timeout
  //teststr.trim();                        // remove any \r \n whitespace at the end of the String
  
}
