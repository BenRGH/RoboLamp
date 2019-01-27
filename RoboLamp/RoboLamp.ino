#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 // this is the 'maximum' pulse length count (out of 4096)
// servomin is 0 degrees, servomax is ~190

// potentiometers
uint16_t potPin = A0;
uint16_t potVal = 500; // ranges from ~3 - 1023
//uint16_t lightPotPin = A1;
//uint16_t lightPotVal = 500; 

uint16_t currPulseLen[4] = {150,150,150}; // current positions for servos

uint8_t modeBtnPin = 2;
uint8_t modeBtnState = 0;
uint8_t saveBtnPin = 3;
uint8_t saveBtnState = 0;
uint8_t loadBtnPin = 4;
uint8_t loadBtnState = 0;

uint16_t currServo = 0;
uint8_t modeLedPin[4] = {10,9,8};

uint8_t rotationSpeed = 5;


void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);

  pinMode(modeBtnPin, INPUT); // high is pressed, low is normal
  pinMode(saveBtnPin, INPUT);
  pinMode(loadBtnPin, INPUT);
  pinMode(modeLedPin[0], OUTPUT);
  pinMode(modeLedPin[1], OUTPUT);
  pinMode(modeLedPin[2], OUTPUT);
  digitalWrite(10, HIGH); // turn led on 
}


void loop() {
  // button press changes joint/servo being controlled
  modeBtnState = digitalRead(modeBtnPin); // read button press
  delay(10); // apparently this helps reading
  if(modeBtnState == HIGH){
    if (currServo == 2){
      digitalWrite(modeLedPin[currServo], LOW); // turn curr led off
      currServo = 0;
      digitalWrite(modeLedPin[currServo], HIGH); // turn new led on
      delay(1000); // time to remove finger
      Serial.println("back to 0");
    }else{
      digitalWrite(modeLedPin[currServo], LOW); 
      currServo++;
      digitalWrite(modeLedPin[currServo], HIGH); 
      delay(1000); // time to remove finger
      Serial.println("next servo");
    }
  }

  // save current position
  saveBtnState = digitalRead(saveBtnPin);
  delay(10);
  if(saveBtnState == HIGH){
    Serial.println("saving...");
    /***
      Need to divide by 4 because servo range is 150-550 
      and each byte of the EEPROM can only hold a value from 0 to 255.
    ***/
    uint8_t saveVal = currPulseLen[0] / 4;
    uint8_t saveVal1 = currPulseLen[1] / 4;
    uint8_t saveVal2 = currPulseLen[2] / 4;

    Serial.println(saveVal);
    Serial.println(saveVal1);
    Serial.println(saveVal2);

    // save to permanent mem
    EEPROM.write(0, saveVal);
    EEPROM.write(1, saveVal1);
    EEPROM.write(2, saveVal2);
  }

  // load saved position
  loadBtnState = digitalRead(loadBtnPin); // read button press
  delay(10); // apparently this helps reading
  if(loadBtnState == HIGH){
    Serial.println("loading...");
    // read from permanent mem
    moveServo(0,int(EEPROM.read(0))*4);
    moveServo(1,int(EEPROM.read(1))*4);
    moveServo(2,int(EEPROM.read(2))*4);
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
      delay(10);
      Serial.println("at lowest");
    }else if((currPulseLen[currServo] - rotationSpeed) > SERVOMIN){
      // moves current servo to its previous position - 10
      moveServo(currServo, currPulseLen[currServo] - rotationSpeed); 
      delay(10);
      Serial.println("-10");
    }
    
  }else if (potVal >= 120){
    // go +10
    if ((currPulseLen[currServo] + rotationSpeed) >= SERVOMAX){
      moveServo(currServo, SERVOMAX); // set to highest angle
      delay(10);
      Serial.println("at highest");
      
    }else if((currPulseLen[currServo] + rotationSpeed) < SERVOMAX){
      // moves current servo to its previous position + 10
      moveServo(currServo, currPulseLen[currServo] + rotationSpeed);
      delay(10);
      Serial.println("+10");
    }
    
  }else{
    // deadzone so change nothing
    Serial.println("deadzone");
  }

  // get & set light val
//  lightPotVal = fromPot(analogRead(lightPotPin)); // this should be degrees 0-180
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

