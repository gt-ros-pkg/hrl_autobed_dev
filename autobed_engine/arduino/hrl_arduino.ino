// Define output pin
long myData;
int headUP=5;
int headDN=3;
int bedUP=6;
int bedDN=7;
int legsUP=4;
int legsDN=2;
int ledTimer1;
int pin=13;

#include <SimpleTimer.h>
//#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
//SoftwareSerial XBee(2, 3);

SimpleTimer t1;


void setup() {
  // Initialize the serial
  Serial.begin(9600);
//  XBee.begin(9600);

  ledTimer1 = t1.setInterval(500,resetPIN1);

 
  // Define our pins
  
  pinMode(pin,OUTPUT);digitalWrite(pin,LOW);
  pinMode(headUP, OUTPUT);digitalWrite(headUP, LOW);
  pinMode(headDN, OUTPUT);digitalWrite(headDN, LOW);
  pinMode(bedUP, OUTPUT);digitalWrite(bedUP, LOW);
  pinMode(bedDN, OUTPUT);digitalWrite(bedDN, LOW);
  pinMode(legsUP, OUTPUT);digitalWrite(legsUP, LOW);
  pinMode(legsDN, OUTPUT);digitalWrite(legsDN, LOW);
}

void resetPIN1(){
  digitalWrite(pin, LOW);
  digitalWrite(headUP, LOW);
  digitalWrite(headDN, LOW);
  digitalWrite(bedUP, LOW);
  digitalWrite(bedDN, LOW);
  digitalWrite(legsUP, LOW);
  digitalWrite(legsDN, LOW);
}



void loop() {
  // check to see if there is data waiting
  //if (XBee.available())
  //{
  int avail = Serial.available();

  if(avail > 0) {
    myData = Serial.read();
    
    if(myData == 'A') {
      digitalWrite(pin,HIGH);
      digitalWrite(headUP, HIGH);
      t1.restartTimer(ledTimer1);
    }
    else {digitalWrite(headUP, LOW);}
    
    if(myData == 'B') {
      digitalWrite(pin,HIGH);
      digitalWrite(headDN, HIGH);
      t1.restartTimer(ledTimer1);
    }  
    else {digitalWrite(headDN, LOW);}
    
    if(myData == 'C') {
      digitalWrite(pin,HIGH);
      digitalWrite(bedUP, HIGH);
      t1.restartTimer(ledTimer1);
    }
    else {digitalWrite(bedUP, LOW);}
    
    if(myData == 'D') {
      digitalWrite(pin,HIGH);
      digitalWrite(bedDN, HIGH);
      t1.restartTimer(ledTimer1);
    }
    else {digitalWrite(bedDN, LOW);}
    
    if(myData == 'E') {
      digitalWrite(pin,HIGH);
      digitalWrite(legsUP, HIGH);
      t1.restartTimer(ledTimer1);
    }
    else {digitalWrite(legsUP, LOW);}
    
    if(myData == 'F') {
      digitalWrite(pin,HIGH);
      digitalWrite(legsDN, HIGH);
      t1.restartTimer(ledTimer1);
    }
    else {digitalWrite(legsDN, LOW);}
  }
  //}
t1.run();
}
