#include <motordriver_4wd.h>
#include <seeed_pwm.h>
#include <ChainableLED.h>
 
double LEFT = 0.0;
double RIGHT = 1500.0;
double BOTTOM = 0.0;
double TOP = 1500.0; 
 
int ping( int pingPin );
void Pan( int pos );
 
const int pingPin = 11;
const int buttonPin = SCL;
const int lightsensorPin = A0;
const int servoPin = 13;
 
int button_state = 0;  // state of push button
int lightsensor_state;  // state of light sensor
int pos = 100;     // servo position
 
#include <math.h>
 
// ticks per rotation
#define TPR 72
 
//---- measurements (mm)
#define RW    42.5  // radius wheel
#define D     158.0
const double PIE    = 3.14159265;
const double PIE_O2 = PIE/2.0;
const double PIE2 = PIE*2.0;
 
//---- odometry            
double dtheta = 0.0, theta =  PI/2.0, x = 50.0, y = 50.0, dx = 0.0, dy = 0.0;
double theta_trn;
long left_encoder_count = 0, right_encoder_count = 0;  
int left_dirn = 1, right_dirn = 1;
 
//---- LED
#define NUM_LEDS  1
 
//defines the pin used on arduino.
ChainableLED leds(0, 1, NUM_LEDS);
 
enum {FWD, REV, TRN} state;
 
void setup()
{
    MOTOR.init();
    pinMode(buttonPin, INPUT);
    pinMode(servoPin, OUTPUT);
    attachInterrupt(0, RightEncoder, CHANGE);
    attachInterrupt(1, LeftEncoder, CHANGE);
   
    leds.init();
   
    // go straight
    right_encoder_count = left_encoder_count = 0;
    MOTOR.setSpeedDir1(10, DIRF);
    MOTOR.setSpeedDir2(10, DIRR);
    state = FWD;
    leds.setColorRGB(0, 0, 100, 0);
  
    //Serial.begin(9600);
}
 
void loop()
{ 
  // read sensors 
  int dist_cm = ping(pingPin);  //Serial.println(dist_cm);
  // delay(100);
 
  // update the pose
  Serial.print("left_encoder_count: ");  Serial.print(left_encoder_count); Serial.print(", right_encoder_count: "); Serial.println(right_encoder_count);
 
  if (state == FWD)
  {
    dtheta = PIE * (RW / (D/2.0)) * ((double)(abs(right_encoder_count) - abs(left_encoder_count)) / TPR);
  }
  else
  if (state == TRN)
  {
    if (right_dirn == -1)
      dtheta = PIE * (RW / (D/2.0)) * ((double)(right_encoder_count) / TPR);
    else
      dtheta = PIE * (RW / (D/2.0)) * ((double)(left_encoder_count) / TPR);
  }
 
  theta  = fmod(theta + dtheta, PIE2);
  theta_trn = theta_trn + dtheta;
 
  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;
 
  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;
 
 
  Serial.print("theta: "); Serial.print(theta*180.0/PI);
  Serial.print("dtheta: "); Serial.print(dtheta*180.0/PI);
  Serial.print("  x: "); Serial.print(x);
  Serial.print("  y: "); Serial.println(y);
 
  //---- check for boundaries
  if ((state == FWD) && (x > RIGHT || x < LEFT || y > TOP || y < BOTTOM))
  {
    Serial.println("rev");
    MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);
    delay(100);
    left_dirn = -1; right_dirn = -1;   
    right_encoder_count = left_encoder_count = 0;
    MOTOR.setSpeedDir1(10, DIRR); MOTOR.setSpeedDir2(10, DIRF);
    state = REV;
    leds.setColorRGB(0, 100, 0, 0);
  }
  else
  if ((state == REV) && (x < RIGHT && x > LEFT && y < TOP && y > BOTTOM))
  {
    Serial.println("turn right");
    MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);
    delay(100);
    // turn 90
    theta_trn = 0.0;
    left_dirn = 1; right_dirn = -1;
    right_encoder_count = left_encoder_count = 0;
    MOTOR.setSpeedDir1(15, DIRF); MOTOR.setSpeedDir2(15, DIRF);
    state = TRN;
    leds.setColorRGB(0, 0, 0, 100);
  }
  else
  if ((state == TRN) && (fabs(fabs(theta_trn)-PIE_O2) < 0.3))
  {
    Serial.println("go fwd");
    left_dirn = 1; right_dirn = 1;
    right_encoder_count = left_encoder_count = 0;
    MOTOR.setSpeedDir1(10, DIRF); MOTOR.setSpeedDir2(10, DIRR);
    state = FWD;   
    leds.setColorRGB(0, 0, 100, 0);
  }
  else
  {
    right_encoder_count = left_encoder_count = 0;
    Serial.println("continue");   
  }
}
 
 
int
ping( int pingPin )
{
long duration, inches, cm;
 
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
 
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
 
  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration); 
  delay(100);
 
  return cm;
}
 
long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}
 
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
 
 
 
void LeftEncoder()
{
  left_encoder_count = left_encoder_count + left_dirn;
}
 
void RightEncoder()
{
  right_encoder_count = right_encoder_count + right_dirn;
}
