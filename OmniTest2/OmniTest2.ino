#include <Wire.h>
#include <Servo.h>
#include "configure.h"
#include "ServoDecode.h"

int courseTheta = 0;
int courseMagnitude = 0;
        
int X  = 0;
int Y  = 0;
int ROT = 0;
int AUX = 0;
bool L_UP_pushed = false;
bool L_DWN_pushed = false;
bool R_UP_pushed = false;
bool R_DWN_pushed = false;

bool RCenabled = false;
    bool printRCenabled = false;
  bool autonomousTestEnabled = true;
    bool encoderTest = true;

int8_t pitch, roll;                          // Stores pitch and roll values of CMPS10,  signed value
uint16_t bearing, fine;                      // bearing stores integer part of bearing, fine stores decimal part of bearing



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
            digitalWrite(LED_PIN, LOW);
         }	  
	setAutoSpeedRegulationOn(PRIMARY_MD25_ADDR);
	setAutoSpeedRegulationOn(SECONDARY_MD25_ADDR);
	setTimeoutOff(PRIMARY_MD25_ADDR);
	setTimeoutOff(SECONDARY_MD25_ADDR);
	setMD25SpeedByteFormat(PRIMARY_MD25_ADDR, 1);
	setMD25SpeedByteFormat(SECONDARY_MD25_ADDR, 1);
        setAccelerationBoth(ACCELERATION);

	armServo.attach(ARM_SERVO_PIN);
	gripServo.attach(GRIPPER_SERVO_PIN);
	armServo.write(90); // center servos
	gripServo.write(90);	
}

void loop()
{  
  if(RCenabled)
  {
	  if(printRCenabled)
	  {printRC();}
	  else
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
  }
  else if(autonomousTestEnabled)
  {
    if(encoderTest) // demonstrates navigation by dead reckoning, wheel encoders
    {
      if(sinceCompassRead > 14)
      serialchartEncoders();
      /*
      int i;
            
      for(i = -128; i<=127; i++)
      {
        stop;
        encoderResetBoth();
        //delay(25 * i / ACCELERATION);
        drive4wheelSpeeds(i, i, i, i);
        delay(33);
        drive4wheelSpeeds(0, 0, 0, 0);
        delay(700);
        Serial.print("Speed ");
        Serial.print(i);
        
        Serial.print(" /Front :");
        Serial.print(getFrontEnc());
        Serial.print(" Rear :");
        Serial.print(getRearEnc());
        Serial.print(" Left :");
        Serial.print(getLeftEnc());
        Serial.print(" Right :");
        Serial.print(getRightEnc());
        
        Serial.println(" Counts Minimum ");
      }
      drive4wheelSpeeds(10, 10, 10, 10);
      delay(1);
      drive4wheelSpeeds(0, 0, 0, 0);
      delay(500);
      //countsToAccelerateForward(MIN_SPEED, minAccelCount, minDecelCount);
      //countsToAccelerateForward(MID_SPEED, midAccelCount, midDecelCount);
      //countsToAccelerateForward(MAX_SPEED, maxAccelCount, maxDecelCount);
      Serial.print("minAccelCount: ");
      Serial.println(minAccelCount);
      Serial.print("minDecelCount: ");
      Serial.println(minDecelCount);
      Serial.print("midAccelCount: ");
      Serial.println(midAccelCount);
      Serial.print("midDecelCount: ");
      Serial.println(midDecelCount);
      Serial.print("maxAccelCount: ");
      Serial.println(maxAccelCount);
      Serial.print("maxDecelCount: ");
      Serial.println(maxDecelCount);
      octagonEncoderInches(6.0); // drive 6 inches in each direction.  If there is no wheel slip, robot should end in the same position it starts.
      openGrip();
      raiseArm();
      lowerArm();
      closeGrip();
      delay(500); // pause to indicate the end of a loop, before another starts
    */
    }
    else // simple timed movement demonstrates all mechanical systems
    {
      octagon(500); // drive in each direction for 1/2 second.  If there is no wheel slip, robot should end in the same position it starts.
      openGrip();
      raiseArm();
      lowerArm();
      closeGrip();
      delay(500); // pause to indicate the end of a loop, before another starts
    }
  }
  else
  {
    // Run in competition mode
    
  
  }
	// speed  -128 (Full Left)   0 (Stop)   127 (Full Right)
	// speed  -128 (Full Reverse)   0 (Stop)   127 (Full Forward)
	// speed  -128 (Full ClockWise)   0 (Stop)   127 (Full CounterClockwise)

}
