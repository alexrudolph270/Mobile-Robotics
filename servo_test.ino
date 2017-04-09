    #include "motordriver_4wd.h"
    #include <seeed_pwm.h>
    #include <time.h>

    #define START_ANGLE   0
    #define END_ANGLE     180
    #define PAN_DELAY     1000
    
    #define SERVO_PULSE_MIN   0.4 * 1000
    #define SERVO_PULSE_MAX   2.4 * 1000
    #define SERVO_PERIOD      25 * 1000
   
    // include button pin
    const int buttonPin = SCL;
    const int pingPin = 11;
    int button_state; // button state var
    //void Pan(int pos);
    //void Scan();

    // Servo defs
    const int servoPin = 2;
    // end servo defs
    
    // Method for reading distance with the ultrasonic ranger
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
      cm = duration/29.0/2.0;
      delay(100);
      return cm;
    }

    ///// SERVO DEFINITIONS + METHODS /////

    void pulsePanTo(float pulseLength)
    {
    const int period = 25 * 1000;
  
    for (int i = 0; i < 200; i++) 
     {
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(pulseLength);
      digitalWrite(servoPin, LOW);
      delayMicroseconds(period - pulseLength);
     }
    }
    

    void anglePanTo(int angle)
    {
      float temp = (angle - START_ANGLE);
      temp = temp * (SERVO_PULSE_MAX -SERVO_PULSE_MIN);
      temp = temp / (END_ANGLE - START_ANGLE);
      temp = temp + SERVO_PULSE_MIN;

      pulsePanTo(temp);
    }

    void panLoop() 
    {
      anglePanTo(START_ANGLE);
      delay(PAN_DELAY);
    
      anglePanTo(END_ANGLE);
      delay(PAN_DELAY);
    }

    ///// END SERVO DEFINITIONS + METHODS /////

    void setup()
    {
        MOTOR.init(); //Init all pin
        pinMode(buttonPin, INPUT); 
        pinMode(servoPin, OUTPUT);
        //defaultPan();
        
        Serial.begin(9600);
    }

    void loop()
    {
        // ping function
        int x = Ping(pingPin);
        Serial.println(x);
        // End ping function

        // Snoot the boop protocol
        ///pinMode(buttonPin, INPUT);
        button_state = digitalRead(buttonPin);
        // Serial.println(button_state);

        if(button_state || x < 20)
        {
          MOTOR.setSpeedDir1(10, DIRF); //Set motor 1 and motor 2 direction:DIRF, Speed:80 (range:0-100).
          //delay(1000);
          MOTOR.setSpeedDir2(10, DIRR); //Set motor 1 and motor 2 direction:DIRR, Speed:80 (range:0-100).
          delay(1000);

          delay(1000);

          ////
          panLoop();
          anglePanTo(90);
          delay(750);
          ////
        }
        else
        {
         MOTOR.setSpeedDir1(10, DIRR); //Set motor 1 and motor 2 direction:DIRF, Speed:80 (range:0-100).
        
         MOTOR.setSpeedDir2(10, DIRF); //Set motor 1 and motor 2 direction:DIRR, Speed:80 (range:0-100).
         delay(1000);
         
        }
    }
