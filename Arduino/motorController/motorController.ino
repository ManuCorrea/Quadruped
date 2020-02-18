/*
 * Valid with L293
 */

int const nMotors = 12;

int motorPins[nMotors];

void setupMotorControl(int nMotors){
  int numberOfMotor = 0;
  // I started at pin 22 but you can start at anyone taking care of not having conflicts with others
  for (int i=22; i<22+nMotors*2; i++){
    pinMode(i, OUTPUT);
    if(i%2 == 0){                   // Each motor needs 2 pins for controlling
      motorPins[numberOfMotor] = i; // we save n knowing that n+1 will be the other
      numberOfMotor ++;
    }
  }
}

// motorDirection negative Right | 0 stop | positive Left 
void motorControl(int motor, int motorDirection){
  if (motorDirection<0){
    digitalWrite(motorPins[motor], LOW);
    digitalWrite(motorPins[motor]+1, HIGH);
  } else if((motorDirection>0)){
    digitalWrite(motorPins[motor], HIGH);
    digitalWrite(motorPins[motor]+1, LOW);
  }else{
    digitalWrite(motorPins[motor], LOW);
    digitalWrite(motorPins[motor]+1, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
