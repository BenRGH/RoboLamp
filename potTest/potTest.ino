
void setup() {
  Serial.begin(9600);
  Serial.println("Start");
}

void loop() {
  // read the value from the sensor:
  uint16_t sensorValue = analogRead(A0);
  delay(200);
  uint16_t sensorValue1 = analogRead(A1);
  delay(200);
  uint16_t sensorValue2 = analogRead(A2);
  delay(200);
  uint16_t sensorValue3 = analogRead(A3);
  delay(200);

  Serial.println(sensorValue);
  Serial.println(sensorValue1);
  Serial.println(sensorValue2);
  Serial.println(sensorValue3);
}
