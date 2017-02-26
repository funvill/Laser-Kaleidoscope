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

// Consts 
// ============================================================================
static const char * SKETCH_VERSION = "0.1" ; 
static const char * SKETCH_LAST_UPDATED = "2017-Feb-26" ; 

// Settings 
// ============================================================================
static const unsigned short SETTING_MAX_NODES = 16 ; 
static const unsigned short SETTING_MOVEMENT_DELAY = 1000*1 ; 
static const float SETTING_MOVEMENT_RATE = 0.2f ; 




// called this way, it uses the default address 0x40
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver( 0x40);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)


float gTimesTable ; 

float CalculateAngle( unsigned short nodeOffset, unsigned short nodeCount, float timesTable  ) {
  
  // Check bounderies 
  if(nodeOffset > nodeCount ) {
		return 0.0f; 
	}

	// Calulate what node to look at.
	float pointingOffset = (nodeOffset * timesTable);
	float targetPoint = fmod( pointingOffset, nodeCount);
	float deltaPoint = fmod( (pointingOffset - nodeOffset), nodeCount);

	// Check for no movment.
	if( deltaPoint == 0.0f ) {
		// Nothing to do, No movement. 
		return 0.0f; 
	}
	
	// I don't understand any of this. This was made by @Luthor2k
	// https://github.com/funvill/mathclock/commits/master?author=Luthor2k
	// It finds the angle between to points on a circle. Basic grade 11 math. 
	// It works! 
	float beta = deltaPoint * ((2 * PI) / nodeCount);
	float rho = atan(sin(beta) / (1 - cos(beta)));
	float theta = (PI / 2) - rho;
	float thetaDegrees = (theta / PI) * 180;

	// Print it for debug
	// printf("[%02d], %.1f (%.1f),\t%.1f\n", nodeOffset, targetPoint, deltaPoint, thetaDegrees);

	return thetaDegrees ; 
}

void setup() 
{
  Serial.begin(115200);
  Serial.println("Version: " + String( SKETCH_VERSION ) ) ;  
  Serial.println("Last updated: " + String( SKETCH_LAST_UPDATED ) ) ;  

  // Start the times tables off at low number. 
  gTimesTable = 1.0f;
  
  // Start up the servos.
  pwm.begin();  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() 
{
  // Update the times table. 
  gTimesTable += SETTING_MOVEMENT_RATE ; 
  Serial.println("TimesTable=" + String( gTimesTable ) ) ;  

  // Move the servos.
  for( unsigned short servonum = 0 ; servonum < SETTING_MAX_NODES ; servonum++ ) {
    Serial.print(servonum);
    Serial.print(" = ");

    float degrees = CalculateAngle( servonum+1, SETTING_MAX_NODES, gTimesTable ); 
		Serial.print(degrees);
		Serial.println("");

    // int pulselen = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
    // pwm.setPWM(servonum, 0, pulselen);
  }

  // Delay between movements
  delay(SETTING_MOVEMENT_DELAY);
  Serial.println(""); 
}
