#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
int pulselength;

const int MPU_addr = 0x68; // AD0 low. look at "9.2 I2C Interface" in datasheet
int16_t AcX, AcY, AcZ; // 16bit precision from MPU 6000/6050
int minVal = 265; int maxVal = 402;

double x; double y; double z;

void setup_MPU(){ // Process needed to setup MPU
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void XYdata(double *x, double *y){ // Function to get x and y angles from MPU
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  *x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  *y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
}

void setup() {
  setup_MPU();
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
} 
void loop() {
  // TODO: add MPU interruptions for more efficient process
  XYdata(&x, &y);
  
  if(x>0 && x<90){
    pulselength = map(int(map(x,90, 0, 0, 90)), 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(1, 0, pulselength);
  }else if(x<360 && x>270){
    pulselength = map(int(map(x,360, 275, 90, 180)), 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(1, 0, pulselength);
  }

  if(y>0 && y<90){
    pulselength = map(int(map(y,90, 0, 180, 90)), 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(0, 0, pulselength);
  }else if(y<360 && y>270){
    pulselength = map(int(map(y,360, 275, 90, 0)), 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(0, 0, pulselength);
  }
  delay(100);
}
