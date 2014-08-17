/*
  AnalogReadSerial
 Reads an analog input on pin 0, prints the result to the serial monitor 
 
 This example code is in the public domain.
 */

void setup() {
  Serial.begin(115200);
}

void loop() {
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);
  int a3 = analogRead(A3);
  Serial.print(a0);
  Serial.print(',');
  Serial.print(a1);
  Serial.print(',');
  Serial.print(a2);
  Serial.print(',');
  Serial.println(a3);
  delay(10);
}
