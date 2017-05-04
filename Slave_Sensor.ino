  #include <time.h>
#include <Wire.h>
#include "motordriver_4wd.h"
#include "seeed_pwm.h"
#include "math.h"

// Slave code!!!
// The Arduino Mega 2560 will be the slave sender 
// The Arduino 320 will be the master receiver

const int sensorPin_F = 8;
const int sensorPin_L = 6;
//const int sensorPin_R = 14;
const int trigPin = 15;
const int echoPin = 16;

int Ping(int SensorPin)
{
  long duration, cm;
  pinMode(SensorPin, OUTPUT);
  digitalWrite(SensorPin, LOW);
  delayMicroseconds(2);
  digitalWrite(SensorPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(SensorPin, LOW);
  pinMode(SensorPin, INPUT);
  duration = pulseIn(SensorPin, HIGH);
  cm = duration/29.0/2.0;
  delay(100);
  return cm;
}

int rightPing(int TrigPin, int EchoPin)
{
  long duration, cm;
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  //
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  //
  duration = pulseIn(echoPin, HIGH);
  //
  cm = duration/29.0/2.0;
  delay(100);
  return cm;
}

void setup() {
  // put your setup code here, to run once
  // Slave -> Master communication
  Wire.begin(8);  // address
  Wire.onRequest(requestEvent);
  //
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //
  Serial.begin(9600);
}

void loop()
{
  delay(100);
  //requestEvent();
}

void requestEvent() {
  // put your main code here, to run repeatedly:
  int f = Ping(sensorPin_F);
  //Serial.print("FRONT:"); Serial.println(f);

  int l = Ping(sensorPin_L);
  //Serial.print("LEFT:"); Serial.println(l);

  int r = rightPing(trigPin, echoPin);
  //Serial.print("RIGHT:"); Serial.println(r);

  Wire.write(f);
  Wire.write(l);
  Wire.write(r);
  
}

