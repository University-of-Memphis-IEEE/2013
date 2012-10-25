#include <Wire.h>
// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <math.h>

//pins
static const uint8_t COURSE_THETA_PIN = A4;
static const uint8_t COURSE_MAGNITUDE_PIN = A5;

// The primary purpose of this processor is to determine and make available these primary variables
int courseX = 0;  // position of the center of the robot along the long axis of the course
int courseY = 0;  // position of the center of the robot along the short axis of the course
int gripX = ;  // position of the gripper along the long axis of the course
int gripY = ;  // position of the gripper along the short axis of the course
int orientation = ;  //orientation of the gripper.  0 is pointing due west, 90 is due north, 180 is due east, 270 is due south. 
// Orientation should never be much less than 0 or much greater than 270 to avoid interference with pot mechanical ends and grip arm contact with tape.
// Cardinal direction definitions
// From the perspective of the robot in the starting square
// North is in the direction of the uppermost level of the ramp.
// East is toward the start of the ramp.

// primary variables support the following functions running on the robot processor
// driveBotTo(int courseX, int courseY, int orientation)
// driveGripTo(int gripX, int gripY, int orientation)

// secondary constants and variables needed to compute primary variables
static const int bot2gripDistance = 48; // confirm as built distance in 1/8ths of an inch (aerial) from robot center to point between gripper fingers
static const int startMagnitudeCount = 32; // distance from initial robot center to pivot of tape measure.  confirm as built distance in 1/8ths of an inch

static const long zeroAngleValueCourse = 60; // confirm as built, the ADC value of the pot when tape alligned with X axis
static const long rightAngleValueCourse = zeroAngleValue+341; // confirm as built, the ADC value of the pot when tape alligned with Y axis
                    // with 
static const long zeroAngleValueBot = 60; // confirm as built, with the ADC value of the pot when tape alligned with X axis
static const long rightAngleValueBot = zeroAngleValue+341; // confirm as built, the ADC value of the pot when tape alligned with Y axis

long magnitudeCount = startMagnitudeCount; // at any time, represents the linear distance in 1/8ths of an inch from robot center to pivot of tape measure

int ThetaADC

double courseThetaRadian = 0; // at any time, represents the angle in radians between the X axis and the tape.



void setup()
{
	Wire.begin();  
	Serial.begin(9600);
        pinMode(6, OUTPUT);
	digitalWrite(6, HIGH);
}

void loop()
{ 
  courseThetaRadian = map(analogRead(COURSE_THETA_PIN), zeroAngleValue, rightAngleValue, 0, M_PI_2);
  courseMagnitude =
  courseX = courseMagnitude * cos(courseTheta);
  courseY = courseMagnitude * sin(courseTheta);
  
}

void cart2polar(long X, long Y, long )
{

}

void polar2cart()
{

}

//***********TRIG LOOKUP TABLES**********
//returns a trig sin or cos calculation value multiplied by 100 (to avoid floating point math)
//returns a trig tan calculation value multiplied by 10 (to avoid floating point math)
//only allows for angles between 0 and 360 degrees

//multiplied by 100 so no floating point math
signed int angtable[73]={100,100,98,97,94,91,87,82,77,71,64,57,50,42,34,26,17,9,0,-9,-17,-26,-34,-42,-50,-57,-64,-71,-77,-82,-87,-91,-94,-97,-98,-100,
						 -100,-100,-98,-97,-94,-91,-87,-82,-77,-71,-64,-57,-50,-42,-34,-26,-17,-9,0,9,17,26,34,42,50,57,64,71,77,82,87,91,94,97,98,100,100};

signed int cos_SoR(long signed int degrees)//returns cos*100
	{
	if (degrees >= 0)//positive angles
		return angtable[degrees/5];
	else
		return -angtable[72-(-degrees)/5];
	}

signed int sin_SoR(long signed int degrees)//returns sin*100
	{
	degrees=degrees - 90;//phase shift 90 degrees

	if (degrees >= 0)//positive angles
		return angtable[degrees/5];
	else
		return -angtable[72-(-degrees)/5];
	}

signed int tan_SoR(long signed int degrees)//returns tan * 10
	{
	//tan(x) = sin(x)/cos(x)
	if (degrees == 90 || degrees == -90 || degrees == 270 || degrees == -270)//blows up
		return 0;//what else should I return?!?!?
	return sin_SoR(degrees)/cos_SoR(degrees)*10;
	}

//***************************************
