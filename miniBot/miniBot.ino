#include <Servo.h>
int servoTmp = 90;

// motor defines
#define leftMotorGreen      0
#define leftMotorRed        1
#define leftMotorPWMpin     9
#define rightMotorGreen     2
#define rightMotorRed       3
#define rightMotorPWMpin   10
#define LED_PIN 11
#define DUMP_SERVO_PIN 4
Servo dumpServo;
bool RCenabled = false;
bool R_UP_pushed = false;
bool R_DWN_pushed = false;	
bool configureRCenabled = false;

void drive(int leftSpeed, int rightSpeed) // positive values forward, negative reverse.  -127 -0- +127
{  
  digitalWrite(leftMotorGreen, leftSpeed <= 0);
  digitalWrite(leftMotorRed, leftSpeed >= 0);
  
  digitalWrite(rightMotorGreen, rightSpeed <= 0);
  digitalWrite(rightMotorRed, rightSpeed >= 0);
  
  analogWrite(leftMotorPWMpin, abs(leftSpeed)*2);
  analogWrite(rightMotorPWMpin, abs(rightSpeed)*2);
}
// for boards other than Teensy 2.0 must determine ICP pin
#define ICP_PIN 22
#include "ServoDecode.h"

void setup()
{
    pinMode(leftMotorGreen, OUTPUT);
    pinMode(leftMotorRed, OUTPUT);
    pinMode(leftMotorPWMpin, OUTPUT);

    pinMode(rightMotorGreen, OUTPUT);
    pinMode(rightMotorRed, OUTPUT);
    pinMode(rightMotorPWMpin, OUTPUT);
    Serial.begin(9600);
ServoDecode.begin();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);        
    delay(2000);
if( ServoDecode.getState()== READY_state)// check if RC is hooked up and turned on.
{
  RCenabled = true;
  configureRC(configureRCenabled);
  digitalWrite(LED_PIN, LOW);
}

    dumpServo.attach(DUMP_SERVO_PIN);
    dumpServo.write(0); // retract servo
}

void loop()
{
  if(RCenabled)
  {
    // Run in radio controlled mode     
    drive(map(ServoDecode.GetChannelPulseWidth(3), 2000, 1000, -127, 127), map(ServoDecode.GetChannelPulseWidth(2), 2000, 1000, -127, 127));
      if(R_UP_pushed = (ServoDecode.GetChannelPulseWidth(6) < 1500-200) &&
	 (servoTmp = dumpServo.read()) < 180)// dumpServo extend commanded and not already fully extended
	{dumpServo.write(servoTmp+1);}
      if(R_DWN_pushed = (ServoDecode.GetChannelPulseWidth(6) > 1500+200) &&
	 (servoTmp = dumpServo.read()) > 0)// dumpServo retract commanded and not already fully retracted
	{dumpServo.write(servoTmp-1);}  
	
	//illuminate LED on valid button push
      if(R_UP_pushed || R_DWN_pushed)
	{digitalWrite(6, HIGH);}                                                                                                                                                                                                                               
	else
	{digitalWrite(6, LOW);} 
  }
  else
  {
    // Run in competition mode
    drive(64, 64);
    delay(2000);
    drive(-64, -64);
    delay(2000);
    drive(50, -50);
    delay(500);
    drive(0, 0);
  
  }
}



