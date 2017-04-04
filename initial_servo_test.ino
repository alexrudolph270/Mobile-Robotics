#include <Servo.h>
#include "motordriver_4wd.h"
#include "seeed_pwm.h"

int Ping(int pingPin);
const int pingPin = 10;
const int pingPin = 10;
const int pingPin = 10;
const int pingPin = 10;

Servo myservo;

int pos = 0;

int Ping(int pingPin)
{
  long duration, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  cm = duration/29/2;
  delay(100);
  return cm;
}

void setup() {
  myservo.attach(2);
}

void loop()
{
    pos = 0;
    myservo.write(pos);
    delay(1000);

    pos = 90;
    myservo.write(pos);
    delay(1000);

    pos = 180;
    myservo.write(pos);
    delay(1000);
}
