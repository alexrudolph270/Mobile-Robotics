#include <motordriver_4wd.h>
#include <seeed_pwm.h>
#include <ChainableLED.h>
#include <Wire.h>
#include <time.h>

//-------- PIE constants

const double PIE    = 3.14159265;
const double PIE_O2 = PIE/2.0;
const double PIE2 = PIE*2.0;

//-------- LED variables

#define NUM_LEDS  1
ChainableLED leds(A0, A1, NUM_LEDS);

//-------- motor control 

void TurnLeft90();
void TurnRight90();
void Straight( int speed, int dirn );
void Wander();

//-------- bumper
const int buttonPin = 11;
int button_state = 0;

//-------- ping variables

void Ping();

int dist_front;
int dist_left;
int dist_right;

//-------- dead reckoning 

// ticks per rotation
#define TPR 72
 
//-------- robot measurements (mm)
#define RW    42.5  // radius wheel
#define D     158.0

//-------- robot config variables
double x = 100.0, y = 100.0, dx = 0.0, dy = 0.0;
double theta =  PI/2.0;

//-------- encoder variables
volatile long left_encoder_count = 0, right_encoder_count = 0;   
int left_dirn = 1, right_dirn = 1;

//-------- robot state 
enum {WANDER, RETURN, INCLINE} state;
enum {FWD, REV} wander_state;

//-------- model of environment 

double LEFT = 0.0;
double RIGHT = 1500.0;
double BOTTOM = 0.0;
double TOP = 1500.0;

float ogrid[][5] =  { {0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0},
                      {0.0,0.0,0.0,0.0,0.0} };

float obj_x, obj_y;
int row, col;

//======================================================================================
// setup
//======================================================================================
void setup()
{
    leds.init();
    MOTOR.init();

    Wire.begin();

    attachInterrupt(0, LeftEncoder, CHANGE);
    attachInterrupt(1, RightEncoder, CHANGE);

    pinMode(buttonPin, INPUT);
        
    // go straight
    Straight( 10, 1 ); 
    leds.setColorRGB(0, 0, 100, 0);  // green
    state = WANDER;
    wander_state = FWD;    
       
    Serial.begin(9600);
}

//======================================================================================
// Loop
//======================================================================================
void loop()
{
  delay(100);

  //---- update occupancy grid  
  Ping(); 

  Serial.print("( L:"); Serial.print(dist_left); 
  Serial.print(", F:"); Serial.print(dist_front); 
  Serial.print(", R:"); Serial.print(dist_right);
  Serial.println(" )");
  
//  if (dist_left < 50)
//  {
//    obj_x = x + sin(theta)*dist_left;
//    obj_y = y - cos(theta)*dist_left;
//    
//    row = int(obj_x/30);
//    col = int(obj_y/30);
//    
//    if (row >= 0 && row <= 5 && col >= 0 && col <= 5)  
//    {
//      if (ogrid[row][col] <= 0.0)
//      {
//        ogrid[row][col] = 0.5;
//      }
//      ogrid[row][col] += (1.0 - ogrid[row][col])/2.0;
//    }
//  }

  if (dist_front < 50)
  {
    obj_x = x + sin(theta)*dist_front;
    obj_y = y - cos(theta)*dist_front;
    
    row = int(obj_x/30);
    col = int(obj_y/30);
    
    if (row >= 0 && row <= 5 && col >= 0 && col <= 5)  
    {
      if (ogrid[row][col] <= 0.0)
      {
        ogrid[row][col] = 1;
      }
      // ogrid[row][col] += (1.0 - ogrid[row][col])/2.0;
    }
  }

//  if (dist_right < 50)
//  {
//    obj_x = x + sin(theta)*dist_right;
//    obj_y = y - cos(theta)*dist_right;
//    
//    row = int(obj_x/30);
//    col = int(obj_y/30);
//    
//    if (row >= 0 && row <= 5 && col >= 0 && col <= 5)  
//    {
//      if (ogrid[row][col] <= 0.0)
//      {
//        ogrid[row][col] = 0.5;
//      }
//      ogrid[row][col] += (1.0 - ogrid[row][col])/2.0;
//    }
//  }
  
  //---- update robot config (x,y,theta)
  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;
  
  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;

  Serial.print("X = "); Serial.println(x);
  Serial.print("Y = "); Serial.println(y);
  
  right_encoder_count = left_encoder_count = 0;
  
  //---- a simple two-state behavior to stay in the box and detect bumps

  if( digitalRead(buttonPin) == 1) {button_state = 1;}; 

  //---- simple behavior to wander and go home

  if (state == WANDER)
  {
    //---- try to reach goal cell
    Wander();

    //---- check if we've arrived at goal cell
    if (x > 1400 && y > 1400)
    {
      //---- stop
      Straight( 0, 0 );    
      delay(100);

      //---- update state
      state = RETURN;         
    }
  }
  else
  if (state == RETURN)
  {
    //---- use map to go home 

    //---- stop
    Straight( 0, 0 );    
    delay(100);

    //---- print ogrid
    for (int i = 0; i < 5; i++)
    {
      for (int j = 0; j < 5; j++)
      {
        Serial.print("["); Serial.print(ogrid[i][j]); Serial.print("]");
      }
      Serial.println();
    }   

    //---- update state
    state = INCLINE;
  }
  else
  if (state == INCLINE)
  {
    for (int i = 0; i < 5; i++)
    {
      leds.setColorRGB(0, 100, 0, 0);  // red
      delay(1000);
      leds.setColorRGB(0, 0, 100, 0);  // green
      delay(1000);
      leds.setColorRGB(0, 0, 0, 100);  // blue
      delay(1000);
    } 
  }
}

//======================================================================================
// Wander()
//======================================================================================
void Wander()
{
  if ((wander_state == FWD) && button_state == 1)
  {
    //---- stop
    Straight( 0, 0 );    
    delay(100);

    //---- back up
    leds.setColorRGB(0, 100, 0, 0);  // red
    Straight( 10, -1 );    
    delay(100);

    button_state = 0;

    //---- update state
    wander_state = REV;   
  }
  else if ((wander_state == FWD) && (x >= RIGHT || x <= LEFT || y >= TOP || y <= BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );    
    delay(100);

    //---- back up
    leds.setColorRGB(0, 100, 0, 0);  // red
    Straight( 10, -1 );    
    delay(150);

    //---- update state
    wander_state = REV;    
  }
  else if ((wander_state == REV) && (x < RIGHT && x > LEFT && y < TOP && y > BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );    

    if (theta <= 0.0 && theta >= -PIE_O2)
    {
      //---- turn left 90
      leds.setColorRGB(0, 0, 0, 100);
      TurnLeft90();
      //---- update robot config (theta)
      theta  = fmod(theta + PIE_O2, PIE2);
      delay(100);
    }
    else
    {
      //---- turn right 90
      leds.setColorRGB(0, 0, 0, 100);
      TurnRight90();
      //---- update robot config (theta)
      theta  = fmod(theta - PIE_O2, PIE2);
      delay(100);      
    }

    //---- go straight
    leds.setColorRGB(0, 0, 100, 0);  // green
    Straight( 10, 1 );   

    //---- update state
    wander_state = FWD;    
  }
}


//======================================================================================
// TurnLeft90
//======================================================================================
void
TurnLeft90()
{
    right_encoder_count = left_encoder_count = 0;
    
    left_dirn = -1; right_dirn = 1;
    MOTOR.setSpeedDir1(40, DIRR); MOTOR.setSpeedDir2(40, DIRR); 
    while (right_encoder_count < 70)
    {
      delayMicroseconds(1);
    }

    MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);
}

//======================================================================================
// TurnRight90
// dirn is 1 for right, -1 for left
//======================================================================================
void
TurnRight90()
{
    right_encoder_count = left_encoder_count = 0;
    
    left_dirn = 1; right_dirn = -1;
    MOTOR.setSpeedDir1(40, DIRF); MOTOR.setSpeedDir2(40, DIRF); 
    while (left_encoder_count < 70)
    {
      delayMicroseconds(1);
    }

    MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);
}

//======================================================================================
// Straight
// dirn is 1 for fwd, -1 for bwd
//======================================================================================
void
Straight( int speed, int dirn )
{
    //---- setup encoder variables
    left_dirn = dirn; right_dirn = dirn;
    
    if (speed == 0)       //-- stop
    {
      MOTOR.setStop1(); MOTOR.setStop2();   return;
    }
    else if (dirn == 1)   //-- fwd
    {
      MOTOR.setSpeedDir1(speed+0.1*speed, DIRF); MOTOR.setSpeedDir2(speed, DIRR); 
    }
    else                  //-- bwd
    {
      MOTOR.setSpeedDir1(speed+0.1*speed, DIRR); MOTOR.setSpeedDir2(speed, DIRF); 
    }
}



//======================================================================================
// Interrupt Service Routines for encoders
//======================================================================================
void LeftEncoder()
{
  left_encoder_count = left_encoder_count + left_dirn;
}

void RightEncoder()
{
  right_encoder_count = right_encoder_count + right_dirn;
}

//======================================================================================
// Alex's Ping
//======================================================================================
void Ping()
{
  Wire.requestFrom(8, 3);
  int count = 0;
  
  while(Wire.available())
  {
    int c = Wire.read();
    if(count == 0)
    {
      // Serial.print("FRONT: ");
      dist_front = c;
    }
    else if(count == 1)
    {
      // Serial.print("LEFT: ");
      dist_left = c;
    }
    else if(count == 2)
    {
      // Serial.print("RIGHT: ");
      dist_right = c;
    }
    // Serial.println(c);
    count++;
  }
  delay(50);
}
