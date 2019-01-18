#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 // this is the 'maximum' pulse length count (out of 4096)
// servomin is 0 degrees, servomax is ~190

// potentiometers
uint16_t potPin = A0;
uint16_t potVal = 500; // ranges from ~3 - 1023
uint16_t lightPotPin = A1;
uint16_t lightPotVal = 500; 

uint16_t currPulseLen[4] = {150,150,150,150}; // current positions for servos

uint16_t modeBtnPin = 2;
int modeBtnState = 0;
uint16_t currServo = 0;

uint8_t rotationSpeed = 5;


void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);

  pinMode(modeBtnPin, INPUT); // high is pressed, low is normal

}


void loop() {
  // button press changes joint/servo being controlled
  modeBtnState = digitalRead(modeBtnPin); // read button press
  delay(10); // apparently this helps reading
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
  }
  
  delay(100);

  // If pot is right then target +10, if left then target -10
  // deadzone is 60-120 degrees
  potVal = fromPot(analogRead(potPin)); // this should be pot degrees 0-180
  delay(10); 
  Serial.println("degrees:");
  Serial.println(potVal);
  if (potVal <= 60){ 
    // go -10
    if ((currPulseLen[currServo] - rotationSpeed) <= SERVOMIN){
      moveServo(currServo, SERVOMIN); // set to lowest angle
      delay(100);
      Serial.println("at lowest");
    }else if((currPulseLen[currServo] - rotationSpeed) > SERVOMIN){
      // moves current servo to its previous position - 10
      moveServo(currServo, currPulseLen[currServo] - rotationSpeed); 
      delay(100);
      Serial.println("-10");
    }
    
  }else if (potVal >= 120){
    // go +10
    if ((currPulseLen[currServo] + rotationSpeed) >= SERVOMAX){
      moveServo(currServo, SERVOMAX); // set to highest angle
      delay(100);
      Serial.println("at highest");
      
    }else if((currPulseLen[currServo] + rotationSpeed) < SERVOMAX){
      // moves current servo to its previous position + 10
      moveServo(currServo, currPulseLen[currServo] + rotationSpeed);
      delay(100);
      Serial.println("+10");
    }
    
  }else{
    // deadzone so change nothing
    Serial.println("deadzone");
  }

  // get & set light val
//  lightPotVal = fromAnalogue(analogRead(lightPotPin)); // this should be degrees 0-180
//  delay(10); // apparently this helps reading
//  Serial.println("light intensity degree: "); 
//  Serial.println(lightPotVal);
//  delay(100);

}


uint16_t fromDegrees(uint16_t degree){
  // returns the servo pulselength for the given degrees
  uint16_t pulseLength = map(degree, 0, 180, SERVOMIN, SERVOMAX);
  return pulseLength;
}

uint16_t fromPot(uint16_t analogue){
  // returns the degrees for the given pot
  uint16_t degree = map(analogue, 0, 1023, 0, 180);
  return degree;
}


void moveServo(uint16_t servo, uint16_t targetPulseLength){
  Serial.println("pl target:");
  Serial.println(targetPulseLength);
  
  if (currPulseLen[servo] < targetPulseLength){
    //Serial.println("going up");
    for (uint16_t pulselen = currPulseLen[servo]; pulselen < targetPulseLength; pulselen++) {
      pwm.setPWM(servo, 0, pulselen); 
      // first arg is channel/servo, ignore second, last is degree
    }
    currPulseLen[servo] = targetPulseLength;
  } else if(currPulseLen[servo] > targetPulseLength){
    //Serial.println("going down");
    for (uint16_t pulselen = currPulseLen[servo]; pulselen > targetPulseLength; pulselen--) {
      pwm.setPWM(servo, 0, pulselen);
    }
    currPulseLen[servo] = targetPulseLength;
  }
  
}

