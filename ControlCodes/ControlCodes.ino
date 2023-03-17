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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
  // cmd = Serial.readStringUntil('\r').;
  cmd =  Serial.readStringUntil('\r').toInt();
  // Serial.print("cmd =" + cmd);
  // Serial.print("cmdFlag =" + cmdFlag);
  digitalWrite(forwardPin, HIGH);
  digitalWrite(backwardPin, LOW);
  digitalWrite(leftPin, HIGH);
  digitalWrite(rightPin, LOW);
  digitalWrite(upPin, LOW);
  digitalWrite(divePin, HIGH); 
  switch (cmd){
    case 10:
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH); 
      cmdFlag = cmd;
      delay(cmdTime);
      break;
    case 2:
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(150);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, HIGH);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(50);
             
      // Serial.print("cmd =" + cmd);
      // Serial.print("cmdFlag =" + cmdFlag);
      break;
    case 1:
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
    case 3:
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(220);
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, HIGH);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(200);
      break;
    case 0:
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, HIGH);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(150); // Limitation is 100ms and cannot be less
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, LOW);
      digitalWrite(divePin, HIGH);
      delay(160);
      break;
    case 4:
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      digitalWrite(upPin, HIGH);
      digitalWrite(divePin, HIGH);
      cmdFlag = cmd;
      delay(cmdTime);
      break;
    case 5:
      downNum = 20;
      upNum = 3;
      int count = 0;
      while (count < 10){
        int stableDownNum = 5;
        int stableUpNum = 3;
        for (int j=0; j<stableDownNum; j++){
          digitalWrite(forwardPin, HIGH);
          digitalWrite(backwardPin, LOW);
          digitalWrite(leftPin, HIGH);
          digitalWrite(rightPin, LOW);
          digitalWrite(upPin, LOW);
          digitalWrite(divePin, LOW);
          delay(120);  
          }
        delay(200);
        for (int j=0; j<stableUpNum; j++){
          digitalWrite(forwardPin, HIGH);
          digitalWrite(backwardPin, LOW);
          digitalWrite(leftPin, HIGH);
          digitalWrite(rightPin, LOW);
          digitalWrite(upPin, HIGH);
          digitalWrite(divePin, HIGH);
          delay(150);  
          }
        delay(200);
        count = count + 1;
        }
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
