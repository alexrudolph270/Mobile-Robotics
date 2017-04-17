#include "motordriver_4wd.h"
#include "seeed_pwm.h"
#include <math.h>

// Definitions
// Ticks per rotation ~ 72 // may need to be fine tuned
#define TPR 72
// Measurements
#define RW 42.5 // wheel radius
#define D 158.0 // Distance
// length of robot ~ 9.5 inches == 24.13 cm
// width of robot ~ 7.7 inches == 19.56 cm

// Declare constants
const double PIE = PI;
const double PIE_02 = PI/2.0;
const double PIE2 = PIE*2.0;

// Odometry
double dtheta = 0.0;
double theta = PI/2.0;
double x = 50.0, y = 50.0, dx = 0.0, dy = 0.0;
double theta_trn;
long left_enc_cnt = 0;
long right_enc_cnt = 0;
long left_dirn = 1;
long right_dirn = 1;

enum {FWD, REV, TRN} state;

// Declare variables
double LEFT = 0.0;
double RIGHT = 1000.0;
double BOTTOM = 0.0;
double TOP = 1000.0;

void setup() {
  // put your setup code here, to run once:
  MOTOR.init();
  //
  //
  attachInterrupt(0, RightEncoder, CHANGE);
  attachInterrupt(1, LeftEncoder, CHANGE);

  //  Straight movement
  right_enc_cnt = left_enc_cnt = 0;
  MOTOR.setSpeedDir1(10, DIRF);
  MOTOR.setSpeedDir2(10, DIRR);

  state = FWD;

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Update position
  Serial.print("LEFT ENCODER: "), Serial.println(left_enc_cnt);
  Serial.print("RIGHT ENCODER: "), Serial.println(right_enc_cnt);

  if(state == FWD)
  {
    dtheta = PIE * (RW / (D/2.0)) * ((double)(abs(right_enc_cnt) - abs(left_enc_cnt)) / TPR);
  }
  else
  {
    if(state == TRN)
    {
      if(right_dirn == -1)
        dtheta = PIE * (RW / (D/2.0)) * ((double)(right_enc_cnt) / TPR);
        else
        {
           dtheta = PIE * (RW / (D/2.0)) * ((double)(left_enc_cnt) / TPR);
        }
    }
  }

  theta = fmod(theta + dtheta, PIE2);
  theta_trn = theta_trn + dtheta;
  //
  dx = PIE * RW * cos(theta) * ((double)(left_enc_cnt + right_enc_cnt) / TPR);
  x = x + dx;

  dy = PIE * RW * sin(theta) * ((double)(left_enc_cnt + right_enc_cnt) / TPR);
  y = y + dy;

  Serial.print("theta: "); Serial.println(theta*180.0/PI);
  Serial.print("dtheta: "); Serial.println(dtheta*180.0/PI);
  Serial.print(" x: "); Serial.println(x);
  Serial.print(" y: "); Serial.println(y);

  // Boundary check
  if((state == FWD) && (x > RIGHT || x < LEFT || y > TOP || y < BOTTOM))
  {
    Serial.println("rev");
    MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);
    delay(100);
    left_dirn = -1;  right_dirn = -1;
    right_enc_cnt = left_enc_cnt = 0;
    MOTOR.setSpeedDir1(10, DIRR); MOTOR.setSpeedDir2(10, DIRF);
    state = REV;
  }
  else
    if((state == REV) && (x < RIGHT && x > LEFT && y < TOP && y > BOTTOM))
    {
      Serial.println("turn right");
      MOTOR.setSpeedDir1(0, DIRF); MOTOR. setSpeedDir2(0, DIRR);
      delay(100);
      // 90 deg turn
      theta_trn = 0.0;
      left_dirn = 1; right_dirn = -1;
      right_enc_cnt = left_enc_cnt = 0;
      MOTOR.setSpeedDir1(15, DIRF); MOTOR.setSpeedDir2(15, DIRF);
      state = TRN;
      // LEDS
    }
  else
    if ((state == TRN) && (fabs(fabs(theta_trn)-PIE_02) < 0.3))
    {
      Serial.println("go fwd");
      left_dirn = 1; right_dirn = 1;
      right_enc_cnt = left_enc_cnt = 0;
      MOTOR.setSpeedDir1(10, DIRF); MOTOR.setSpeedDir2(10, DIRR);
      state = FWD;
      // LEDS
    }
    else
    {
      right_enc_cnt = left_enc_cnt = 0;
      Serial.println("continue");
    }
}

// Encoder methods
void LeftEncoder()
{
  left_enc_cnt = left_enc_cnt + left_dirn;
}

void RightEncoder()
{
  right_enc_cnt = right_enc_cnt + right_dirn;
}

