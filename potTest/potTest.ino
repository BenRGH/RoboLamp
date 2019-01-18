uint16_t sensorValue = 500;
uint16_t pin = A0;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(pin);
  delay(200);
  

  Serial.println(sensorValue);
}
