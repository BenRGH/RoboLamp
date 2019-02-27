//LED TEST 2 w/ 74HC595
//by Amanda Ghassaei 2012
//https://www.instructables.com/id/Multiplexing-with-Arduino-and-the-74HC595/

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

//this code sends data to the 74HC595 without "shiftOut"

//pin connections- the #define tag will replace all instances of "latchPin" in your code with A1 (and so on)
#define latchPin A1
#define clockPin A0
#define dataPin A2

//looping variables
byte i;
byte j;

//storage variable
byte dataToSend;

//storage for led states, 4 bytes
byte ledData[] = {15, 15, 15, 15};

void setup() {
  //set pins as output
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

void loop() {
  
  for (i=0;i<4;i++){
    
    //send data from ledData to each row, one at a time
    byte dataToSend = (1 << (i+4)) | (15 & ~ledData[i]);
      
    // setlatch pin low so the LEDs don't change while sending in bits
    digitalWrite(latchPin, LOW);
    
//    // shift out the bits of dataToSend to the 74HC595
//    shiftOut(dataPin, clockPin, LSBFIRST, dataToSend);
// the code below is the equivalent of the two lines above
    for (j=0;j<8;j++){
      digitalWrite(clockPin,LOW);
      digitalWrite(dataPin,((dataToSend>>j)&1));
      digitalWrite(clockPin,HIGH);
    }


    //set latch pin high- this sends data to outputs so the LEDs will light up
    digitalWrite(latchPin, HIGH);
      
  }  
}


