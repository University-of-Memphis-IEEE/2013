#include <Wire.h>
#include <Servo.h>
#include "configure.h"
#include "ServoDecode.h"


long loopCount = 0;
Servo armServo;
Servo gripServo;
int courseTheta = 0;
int courseMagnitude = 0;
        bool RCenabled = false;
        int X  = 0;
	int Y  = 0;
	int ROT = 0;
	int AUX = 0;
	bool L_UP_pushed = false;
	bool L_DWN_pushed = false;
	bool R_UP_pushed = false;
	bool R_DWN_pushed = false;
	
        bool configureRCenabled = false;

void setup()
{
	Wire.begin();  
	Serial.begin(9600);
ServoDecode.begin();
        pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);
        delay(2000);        
if( ServoDecode.getState() == READY_state)// check if RC is hooked up and turned on.
{
  RCenabled = true;
  configureRC(configureRCenabled);
  digitalWrite(LED_PIN, LOW);
}	  
	setAutoSpeedRegulationOn(PRIMARY_MD25_ADDR);
	setAutoSpeedRegulationOn(SECONDARY_MD25_ADDR);
	setTimeoutOff(PRIMARY_MD25_ADDR);
	setTimeoutOff(SECONDARY_MD25_ADDR);
	setMD25SpeedByteFormat(PRIMARY_MD25_ADDR, 1);
	setMD25SpeedByteFormat(SECONDARY_MD25_ADDR, 1);

	armServo.attach(ARM_SERVO_PIN);
	gripServo.attach(GRIPPER_SERVO_PIN);
	armServo.write(90); // center servos
	gripServo.write(90);	
}

void loop()
{  
  if(RCenabled)
  {
    // Run in radio controlled mode
    X  = ServoDecode.GetChannelPulseWidth(4); // left horizontal stick
    Y  = ServoDecode.GetChannelPulseWidth(3); // left vertical stick
    ROT = ServoDecode.GetChannelPulseWidth(1); // right horizontal stick 
    AUX = map(ServoDecode.GetChannelPulseWidth(2), 1000, 2000, -127, 127);
    // do something here with the extra AUX analog channel, right verticle stick.
    moveJoystick(X, Y, ROT);
    parseRCbuttons(L_UP_pushed, L_DWN_pushed, R_UP_pushed, R_DWN_pushed, armServo, gripServo);
  }
  else
  {
    // Run in competition mode
  
  }
	// speed  -128 (Full Left)   0 (Stop)   127 (Full Right)
	// speed  -128 (Full Reverse)   0 (Stop)   127 (Full Forward)
	// speed  -128 (Full ClockWise)   0 (Stop)   127 (Full CounterClockwise)

}

