/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ************************************************** **/

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


  Serial.print("%");
  pwm.begin();

  Serial.print("+");
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  Serial.print("[");
}

void loop() {
  Serial.print(".");
  return ;

  // Do the math and update the angles for each servo 
  // MathClock.Loop();

  // Move the servos.
  for( unsigned short servonum = 0 ; servonum < SETTING_MAX_NODES ; servonum++ ) {
    float degrees = MathClock.GetNodeAngle( servonum ) ; 

    Serial.print(servonum);
		Serial.print("\t");
		Serial.print(degrees);
		Serial.println("");

    int pulselen = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servonum, 0, pulselen);
  }

  // Delay between movements
  delay(1000*10);
}
