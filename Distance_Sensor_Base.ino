#include <time.h>

const int sensorPin = 11;

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int x = Ping(sensorPin);
  Serial.println(x);
  
}

