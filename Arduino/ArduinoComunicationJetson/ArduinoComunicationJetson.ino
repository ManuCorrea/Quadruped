byte comCode = 0;

byte xMov = 2;
byte yMov = 3;


typedef union {
 float Rot;
 byte binary[4];
} binaryFloat;

binaryFloat xRot;
binaryFloat yRot;


int jointStates[12];
int jointCommand[12];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  xRot.Rot = 1.5;
  yRot.Rot = 0.5;
  
  for(int i=0; i<12; i++){
    jointStates[i] = i;
  }
  
  // put your setup code here, to run once:
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

void retRotandMov(){ // 0x05
  Serial.write(xRot.binary, 4);
  Serial.write(yRot.binary, 4);
  Serial.write(xMov);
  Serial.write(yMov);
}

void resetState(){ // 0x03
  // TODO implement reset(all joints to endstops)
}

void setJoints(){ // 0x01
  digitalWrite(LED_BUILTIN, HIGH);
  int angle;
  delay(200); // need to wait to get full array without any problem
  if (Serial.available()>11){
    
    for(int i=0; i<12; i++){
        angle = Serial.read();
        jointCommand[i] = angle;
        jointStates[i] = angle;
      }
  }
  if(Serial.available()){
    if(Serial.read() == 0x02){
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
