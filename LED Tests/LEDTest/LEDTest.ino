// Ben 2019
// Modified from https://www.instructables.com/id/Multiplexing-with-Arduino-and-the-74HC595/

//pins
#define latchPin A1
#define clockPin A0
#define dataPin A2

//storage for led states, 4 bytes
byte ledData[] = {15, 15, 15, 15}; // These are the number of on pins in each row

void setup() {
  //set pins as output
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

void loop() {
  
  for (byte i=0;i<4;i++){
    
    //send data from ledData to each row, one at a time
    byte dataToSend = (1 << (i+4)) | (15 & ~ledData[i]);
      
    // setlatch pin low so the LEDs don't change while sending in bits
    digitalWrite(latchPin, LOW);
    // shift out the bits of dataToSend to the 74HC595
    shiftOut(dataPin, clockPin, LSBFIRST, dataToSend);
    //set latch pin high- this sends data to outputs so the LEDs will light up
    digitalWrite(latchPin, HIGH);

    //delay(10); // Use for strobing, normally off
  }  
}





