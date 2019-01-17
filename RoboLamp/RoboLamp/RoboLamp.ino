#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
// servomin is 0degrees, servomax is ~190

int potPin = A0;
int potVal = 5; // ranges from 3 - 1023
uint16_t currPulseLen = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
}

void loop() {
  Serial.println("reading pot");
  delay(1000);
  potVal = fromAnalogue(analogRead(potPin)); // this should be degrees 0-180
  Serial.println("read pot");
  Serial.println(potVal);
  
  moveServo(0, potVal);
  delay(1000);
  
}

uint16_t fromDegrees(int degree){
  // returns the pulselength for the given degrees
  uint16_t pulseLength = map(degree, 0, 180, SERVOMIN, SERVOMAX);
  return pulseLength;
}

uint16_t fromAnalogue(int analogue){
  // returns the degrees for the given pulselength
  uint16_t degree = map(analogue, 3, 1023, 0, 180);
  return degree;
}

void moveServo(int servo, int degree){
  uint16_t target = fromDegrees(degree); // get servo val from degrees
  Serial.println("target:");
  Serial.println(target);
  
  if (currPulseLen < target){
    Serial.println("going up");
    for (uint16_t pulselen = currPulseLen; pulselen < target; pulselen++) {
      pwm.setPWM(servo, 0, pulselen); 
      // first arg is channel/servo, ignore second, last is degree
    }
    currPulseLen = target;
  } else if(currPulseLen > target){
    Serial.println("going down");
    for (uint16_t pulselen = currPulseLen; pulselen > target; pulselen--) {
      pwm.setPWM(servo, 0, pulselen);
    }
    currPulseLen = target;
  }
  
}

