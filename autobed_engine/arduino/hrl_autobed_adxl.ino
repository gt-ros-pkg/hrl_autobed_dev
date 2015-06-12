/*
  Calibration Code for the Accelerometer
 
 Reads an analog input pin prints to serial monitor
 
 created 29 May. 2015
 by Yash Chitalia
 
 This example code is in the public domain.
 
 */
#include <math.h>
#include <LiquidCrystal.h>
// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin0 = A0;  // Analog input pin that the accelerometer X is attached to
const int analogInPin1 = A1;  // Analog input pin that the accelerometer Y is attached to
const int analogInPin2 = A2;  // Analog input pin that the accelerometer Z is attached to
const int analogInPin3 = A3;  // Analog input pin that the accelerometer X is attached to
const int analogInPin4 = A4;  // Analog input pin that the accelerometer Y is attached to
const int analogInPin5 = A5;  // Analog input pin that the accelerometer Z is attached to
const int BIAS_X = 331;
const int BIAS_Y = 331;
const int BIAS_Z = 260;
int X = 0;
int Y = 0; // value read from the accelerometer
int Z = 0;
float angleValue2 = 0;
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  // set up the LCD's
}

void loop() {
  // set the cursor to column 0, line 1
  lcd.setCursor(0, 1);
  // read the analog in value:
  X1 = (analogRead(analogInPin0) - BIAS_Y);
  Y1 = (analogRead(analogInPin1) - BIAS_Y);
  Z1 = (analogRead(analogInPin2) - BIAS_Z);  
  float angleValue1 = atan2(sqrt(sq(Y1) + sq(X1)), Z1)  * 180.0f / M_PI;
  //float angleValue = atan2(Y, Z)  * 180.0f / M_PI;
  //print the results to the serial monitor:                      
  Serial.print(angleValue1);
  Serial.print(',');
  Serial.println(angleValue2);
  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(10);                     
}
