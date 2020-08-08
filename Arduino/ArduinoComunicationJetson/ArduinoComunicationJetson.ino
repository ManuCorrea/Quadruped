byte comCode = 0;

byte xMov = 2;
byte yMov = 3;


typedef union {
 float Rot;
 byte binary[4];
} binaryFloat;

binaryFloat xRot;
binaryFloat yRot;

// Used to collect the current joint angle values
int jointStates[12];
// Used to collect the desired joint angle values to be sent to motor control board
int jointCommand[12];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  xRot.Rot = 1.5;
  yRot.Rot = 0.5;
  
  // TODO: need to get or reset current joint states
  
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){
    comCode = Serial.read();
    if(comCode==0x00){
      retJointStates();
    }else if(comCode==0x01){
      setJoints();
    }else if(comCode==0x03){
      resetState();
    }else if(comCode==0x05){
      retRotandMov();
    }
  }
}

void retJointStates(){ // 0x00
  for(int i=0; i<12; i++){
    Serial.write(jointStates[i]);
  } 
  Serial.write(0xFF);
}

/*
retRotandMov() is used to return gyroscope and movement data on Serial
*/
void retRotandMov(){ // 0x05
  Serial.write(xRot.binary, 4);
  Serial.write(yRot.binary, 4);
  Serial.write(xMov);
  Serial.write(yMov);
}

/*
resetState() is used to reset the robot to "origin position"
*/
void resetState(){ // 0x03
  // TODO implement reset(all joints to endstops)
}

void setJoints(){ // 0x01
  digitalWrite(LED_BUILTIN, HIGH);
  int angle;
  while(Serial.available() <= 11) {};
  for(int i=0; i<12; i++){
      angle = Serial.read();
      jointCommand[i] = angle;
    }
  if(Serial.available()){
    if(Serial.read() == 0x02){
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  // TODO: send orders to the control board
}
