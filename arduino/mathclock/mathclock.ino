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
  - 16 9g servo motors 
  - 16 line laser pointers. 

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
static const char * SKETCH_VERSION = "0.2" ; 
static const char * SKETCH_LAST_UPDATED = "2017-Mar-10" ; 

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

// The servo starts at 130, but we want to limit the range at 160 to be inline with the first 
// servo in the series. 
static const unsigned short SETTING_SERVOMIN = 160 ; 

// The servo stops at 600, but we want to limit the range at 500 to be inline with the last 
// servo in the series. 
static const unsigned short SETTING_SERVOMAX = 500 ; 

// Analog servos run at ~60 Hz updates
static const unsigned short SETTING_PWM_FREQUENCY = 60 ; 


// Settings 
// ============================================================================
static const unsigned short SETTING_MAX_NODES = 16 ; 
static const unsigned short SETTING_MOVEMENT_DELAY = 1000*1 ; 
static const float SETTING_MOVEMENT_RATE = 0.2f ; 

// All the servos are off by a little bit. This is the difference that they are off by. 
static short SERVO_CALABRATION[SETTING_MAX_NODES] ; 

// Globals 
// ============================================================================
float gTimesTable ; 

// called this way, it uses the default address 0x40
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver( 0x40);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void SetServoDegrees( unsigned char servo, unsigned char deg) {
  if( servo > SETTING_MAX_NODES ) {
    return ; 
  }

  if( deg + SERVO_CALABRATION[servo] > 180 ) {
    deg = 180 ; 
  } else if( deg + SERVO_CALABRATION[servo] < 0 ) {
    deg = 0 ; 
  } else {
    deg += SERVO_CALABRATION[servo] ; 
  }

  int pulselen = map(deg, 0, 180, SETTING_SERVOMIN, SETTING_SERVOMAX);
  pwm.setPWM(servo, 0, pulselen);

  Serial.println("servo: " + String(servo) + ", deg: " + String(deg) + ", SERVO_CALABRATION: "+ String(SERVO_CALABRATION[servo]) ) ;  
}
void CreateServoCalabrationTable() 
{
  // This table is manually generated. 
  // The teeth of the serovs do not let all of the lasers to be places in perfect order. 
  // I used the LasersToCenter() function then check to see how far each laser was off from 
  // pointing at the laser directly across from them. Then I made a guess and updated this table. 
  SERVO_CALABRATION[0]  =  8; 
  SERVO_CALABRATION[1]  = -8; 
  SERVO_CALABRATION[2]  = -8; 
  SERVO_CALABRATION[3]  = -5; 
  SERVO_CALABRATION[4]  = -3; 
  SERVO_CALABRATION[5]  =  5; 
  SERVO_CALABRATION[6]  = -7; 
  SERVO_CALABRATION[7]  = -5; 
  SERVO_CALABRATION[8]  =  8; 
  SERVO_CALABRATION[9]  = -2; 
  SERVO_CALABRATION[10] = 10; 
  SERVO_CALABRATION[11] = -3; 
  SERVO_CALABRATION[12] =  7; 
  SERVO_CALABRATION[13] =  0; 
  SERVO_CALABRATION[14] = -7; 
  SERVO_CALABRATION[15] =  7; 
  
}


void LasersToCenter() {
  int pulselen = 0 ; 
  for( unsigned short node = 0 ; node <= SETTING_MAX_NODES ; node++ ) {
    SetServoDegrees( node, 90 ); 
  }
  delay( 1000) ; 
}


void CalibrateServo() {
    int servo = 0 ; 
    int pulselen = 0 ; 
    Serial.println("CalibrateServo\n=========================" ) ;  
    Serial.println("SERVOMIN: " + String(SETTING_SERVOMIN) + ", SERVOMAX: " + String(SETTING_SERVOMAX) + ", PWM Freq: "+ String(SETTING_PWM_FREQUENCY) ) ;  

    /*
    // Find the min and max pulselen of a servo. 
    // Instructions: 
    // 1. Set the SERVOMIN = 0, and SERVOMAX = 4096.
    // 2. Let the following sequence run, recored when the servo starts moving (min), and when it stops moving (max)
    // 3. Update SERVOMIN and SERVOMAX to match. 
    Serial.println("Detect the SERVOMIN and SERVOMAX" );
    Serial.println("pulselen, degrees" );
    for( unsigned short pulselen = SETTING_SERVOMIN ; pulselen < SETTING_SERVOMAX ; pulselen += 10 ) {
        pwm.setPWM(servo, 0, pulselen);

        unsigned short degrees = map(pulselen, SETTING_SERVOMIN, SETTING_SERVOMAX, 0, 180);
        Serial.println(String(pulselen) + ",\t   " + String(degrees) );
        delay( 500 ); 
    }
    */

    
    // All servos, full scan
    Serial.println("All servos full scan" );
    Serial.println("Servo, degrees, pulselen" );
    for( unsigned short degrees = 0 ; degrees <= 180 ; degrees += 15 ) {
      pulselen = map(degrees, 0, 180, SETTING_SERVOMIN, SETTING_SERVOMAX);
      for( unsigned short node = 0 ; node <= SETTING_MAX_NODES ; node++ ) {        
        pwm.setPWM(node, 0, pulselen);
        Serial.println(String( node ) + ",\t" + String( degrees) + ",\t" + String(pulselen) );
      }
      delay( 100 ); 
    }

   
    return ; 
}


void FullScan() 
{
  unsigned char scanRate = 1 ; 
  unsigned short moveDelay = 1 ; 
  
  // All servos, full scan
  Serial.println("All servos full scan" );

  // Forwards. 
  for( short degrees = 0 ; degrees <= 180 ; degrees += scanRate ) {
    for( unsigned short node = 0 ; node <= SETTING_MAX_NODES ; node++ ) {    
      SetServoDegrees( node, degrees );    
    }
    delay( moveDelay ); 
  }

  // Backwards 
  for( short degrees = 180 ; degrees > 0 ; degrees -= scanRate ) {
    for( unsigned short node = 0 ; node <= SETTING_MAX_NODES ; node++ ) {        
      SetServoDegrees( node, degrees );    
    }
    delay( moveDelay ); 
  }
}

void AllLasersPointAtSameTarget() 
{
  for( unsigned short target = 0 ; target <= SETTING_MAX_NODES ; target++ ) {
    for( unsigned short node = 0 ; node <= SETTING_MAX_NODES ; node++ ) {
  
      float deltaPoint = 0 ; 
      if( target - node < 0 ) {
        deltaPoint = abs( (float) node - target); 
      } else { 
        // Calulate what node to look at.
        deltaPoint = abs( (float) target - node ); 
      }
      
      if( deltaPoint == 0 ) {
        continue; 
      }
      
      float degrees = PointAt( deltaPoint, SETTING_MAX_NODES );
      SetServoDegrees( node, degrees );   
    }
    delay( 500 ); 
  }
}



void LasersToStartingLocations() {
  int pulselen = 0 ; 
  for( unsigned short node = 0 ; node <= SETTING_MAX_NODES ; node++ ) {
    float degrees = PointAt( 1, SETTING_MAX_NODES );
    pulselen = map(degrees, 0, 180, SETTING_SERVOMIN, SETTING_SERVOMAX);
    pwm.setPWM(node, 0, pulselen);
    delay( 500 ); 
  }
}


float PointAt( float deltaPoint, unsigned short nodeCount ) {

	// I don't understand any of this. This was made by @Luthor2k https://github.com/Luthor2k
	// It finds the angle between to points on a circle. Basic grade 11 math. It works! 
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

  CreateServoCalabrationTable(); 

  // Start the times tables off at low number. 
  gTimesTable = 1.0f;
  
  // Start up the servos.
  pwm.begin();  
  pwm.setPWMFreq(SETTING_PWM_FREQUENCY);  

  // Lasers to starting Locations. Pointing at nearest neighbour 
  LasersToStartingLocations(); 
  delay( 1000*1 ); 
  LasersToCenter(); 
  delay( 1000*1 ); 

  
}

unsigned char pattern = 0 ; 
void loop() 
{

  // AllLasersPointAtSameTarget() ; 
  // return ; 
  
  pattern++; 
  
  if( pattern >= 0 && pattern < 5 ) {
    FullScan() ;
  } if( pattern >= 0 && pattern < 5 ) {
    AllLasersPointAtSameTarget() ;
  } else {
    pattern = 0 ; 
  }


  
  return ; 
  
  // CalibrateServo();
  // return ; 

  // Update the times table. 
  gTimesTable += SETTING_MOVEMENT_RATE ; 
  Serial.println("TimesTable=" + String( gTimesTable ) ) ;  

  // Move the servos.
  for( unsigned short servonum = 0 ; servonum < SETTING_MAX_NODES ; servonum++ ) 
  {
    // Calulate what node to look at.
    float pointingOffset = (servonum * gTimesTable);
    float targetPoint = fmod( pointingOffset, SETTING_MAX_NODES);
    float deltaPoint = fmod( (pointingOffset - servonum), SETTING_MAX_NODES);
    if( deltaPoint == 0 ) {
      // We have no where to point at, point at anything
      deltaPoint = 1.0 ; 
    }

    // Calulate the angle.
    float degrees = PointAt( deltaPoint, SETTING_MAX_NODES ); 

    // Move the servo 
    int pulselen = map(degrees, 0, 180, SETTING_SERVOMIN, SETTING_SERVOMAX);
    pwm.setPWM(servonum, 0, pulselen);

    // Debug 
    Serial.println("[" + String( servonum) + "] ==> " + String( targetPoint)  + " ("+ String( deltaPoint ) +"), Angle: " + String( degrees ) ) ;  
  }

  // Delay between movements
  delay(SETTING_MOVEMENT_DELAY);
  Serial.println(""); 
}
