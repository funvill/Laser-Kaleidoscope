/****************************************************************************** 

  Math clock 
  =============================================================================
  Last updated: 2017-Feb-26
  Created by: Steven Smethurst @funvill 

  More information can be found here 
  https://github.com/funvill/mathclock

  Hardware
  -----------------------------------------------------------------------------
  - Adafruit 16-channel PWM & Servo driver http://www.adafruit.com/products/815
  - 96 9g servo motors 
  - 96 line laser pointers. 

  Pin out
  -----------------------------------------------------------------------------
  Arduino Nano      PCA9685 (Adafruit 16-channel PWM & Servo driver) 
  GND           ==> GND 
  Not connected ==> DE 
  A5            ==> SCL
  A4            ==> SDA    
  5V            ==> VCC 
  5V            ==> V+

******************************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "mathclock.h"

// called this way, it uses the default address 0x40
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver( 0x40);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

void setup() {
  Serial.begin(115200);
  Serial.println("16 channel Servo test!");

  pwm.begin();  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() 
{
  Serial.println("TimesTable=" + String( MathClock.GetTimesTable() ) ) ;

  // Do the math and update the angles for each servo 
  MathClock.Loop();

  // Move the servos.
  for( unsigned short servonum = 0 ; servonum < SETTING_MAX_NODES ; servonum++ ) {
    Serial.print(servonum);
    Serial.print("  ");

    float degrees = MathClock.GetNodeAngle( servonum ) ;     
		Serial.print(degrees);
		Serial.println("");

    // int pulselen = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
    // pwm.setPWM(servonum, 0, pulselen);
  }

  // Delay between movements
  delay(1000*10);
}
