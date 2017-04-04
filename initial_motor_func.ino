    #include "motordriver_4wd.h"
    #include <seeed_pwm.h>
    // include button pin
    const int buttonPin = SCL;

    int button_state; // button state var

    void setup()
    {
        MOTOR.init(); //Init all pin
        pinMode(buttonPin, INPUT);
    }

    void loop()
    {
        //MOTOR.setSpeedDir1(10, DIRR); //Set motor 1 and motor 2 direction:DIRF, Speed:80 (range:0-100).
        
        //MOTOR.setSpeedDir2(10, DIRF); //Set motor 1 and motor 2 direction:DIRR, Speed:80 (range:0-100).
        //delay(500);

        // Snoot the boop protocol
        ///pinMode(buttonPin, INPUT);
        button_state = digitalRead(buttonPin);
        Serial.println(button_state);

        if(button_state)
        {
          MOTOR.setSpeedDir1(10, DIRF); //Set motor 1 and motor 2 direction:DIRF, Speed:80 (range:0-100).
        
          MOTOR.setSpeedDir2(10, DIRR); //Set motor 1 and motor 2 direction:DIRR, Speed:80 (range:0-100).
          delay(1000);
        }
        else
        {
          MOTOR.setSpeedDir1(10, DIRR); //Set motor 1 and motor 2 direction:DIRF, Speed:80 (range:0-100).
        
         MOTOR.setSpeedDir2(10, DIRF); //Set motor 1 and motor 2 direction:DIRR, Speed:80 (range:0-100).
          delay(500);
        }
    }
