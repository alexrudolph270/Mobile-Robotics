#include <Servo.h>
#include "motordriver_4wd.h"
#include "seeed_pwm.h"

int Ping(int pingPin);
void Pan(int pos);
void Scan();

const int pingPin = A0;
const int buttonPin = SCL;
// const int photoPin = TBD;
const int servoPin = 2;

Servo myservo;

int button_state;
int pos = 90;

long left_encoder_count = 0;
long right_encoder_count = 0;
int left_dirn = 1;
int right_dirn = 1;

int cm_left, cm_front, cm_right;

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

void Pan(int pos)
{
  myservo.write(pos);
}

void setup() 
{
  MOTOR.init();
  pinMode(buttonPin, INPUT);
  pinMode(servoPin, OUTPUT);
  myservo.attach(2);

  Serial.begin(9600);
}

void loop()
{
    Pan(90); //front
    delay(1000);
    cm_front = Ping(pingPin);
    Serial.print(cm_front);
    Serial.println(" cm(front)");
    delay(100);

    Pan(0); //left
    delay(1000);
    cm_left = Ping(pingPin);
    Serial.print(cm_left);
    Serial.println(" cm(left)");
    delay(100);
    
    Pan(180); //right
    delay(1000);
    cm_right = Ping(pingPin);
    Serial.print(cm_right);
    Serial.println(" cm(right)");
    delay(100);
}
