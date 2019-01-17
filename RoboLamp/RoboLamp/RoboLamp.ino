#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
// servomin is 0degrees, servomax is ~190

// potentiometers
uint16_t potPin = A0;
uint16_t potVal = 5; // ranges from 3 - 1023
uint16_t lightPotPin = A1;
uint16_t lightPotVal = 5; 

uint16_t currPulseLen[4] = {5, 5, 5, 5}; // current positions for servos

uint16_t modeBtnPin = 2;
int modeBtnState = 0;
uint16_t currServo = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);

  pinMode(modeBtnPin, INPUT); // high is pressed, low is normal
}

void loop() {
  // button press changes joint/servo
  modeBtnState = digitalRead(modeBtnPin); // read button press
  if(modeBtnState == HIGH){
    if (currServo == 3){
      currServo = 0;
      delay(1000); // time to remove finger
      Serial.println("back to 0");
    }else{
      currServo++;
      delay(1000); // time to remove finger
      Serial.println("next servo");
    }
    modeBtnState = LOW;
  }

  Serial.println("Servo " + currServo);
  
  delay(100);
  
  potVal = fromAnalogue(analogRead(potPin)); // this should be degrees 0-180
  delay(10); // apparently this helps reading
  Serial.println("degree for servo: " + potVal);
  
  lightPotVal = fromAnalogue(analogRead(lightPotPin)); // this should be degrees 0-180
  delay(10); // apparently this helps reading
  Serial.println("light intensity degree: " + lightPotVal);
  
  delay(10);

  moveServo(currServo, potVal);

  delay(100);

}

uint16_t fromDegrees(uint16_t degree){
  // returns the pulselength for the given degrees
  uint16_t pulseLength = map(degree, 0, 180, SERVOMIN, SERVOMAX);
  return pulseLength;
}

uint16_t fromAnalogue(uint16_t analogue){
  // returns the degrees for the given pulselength
  uint16_t degree = map(analogue, 3, 1023, 0, 180);
  return degree;
}

void moveServo(uint16_t servo, uint16_t degree){
  uint16_t target = fromDegrees(degree); // get servo val from degrees
  Serial.println("target:");
  Serial.println(target);
  
  if (currPulseLen[servo] < target){
    Serial.println("going up");
    for (uint16_t pulselen = currPulseLen[servo]; pulselen < target; pulselen++) {
      pwm.setPWM(servo, 0, pulselen); 
      // first arg is channel/servo, ignore second, last is degree
    }
    currPulseLen[servo] = target;
  } else if(currPulseLen[servo] > target){
    Serial.println("going down");
    for (uint16_t pulselen = currPulseLen[servo]; pulselen > target; pulselen--) {
      pwm.setPWM(servo, 0, pulselen);
    }
    currPulseLen[servo] = target;
  }
  
}

