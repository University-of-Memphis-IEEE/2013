#include <Arduino.h>
Servo armServo;
Servo gripServo;
//Global Configuration Constants
static const uint8_t ACCELERATION = 10; // values from 1 to 10: 
static const uint8_t COUNTS_PER_REV = 360;
static const float WHEEL_DIA = 2.0;
static const int8_t  MIN_SPEED = 10;
static const int8_t  MID_SPEED = 40;
static const int8_t  MAX_SPEED = 127;
static const long minAccelCount = 0;                 // change these to static const once their values are determined.
static const long minDecelCount = 2;
static const long midAccelCount = 11;
static const long midDecelCount = 25;
static const long maxAccelCount = 178;
static const long maxDecelCount = 143;


//Global Variables
enum Wheel_t{FRONT,REAR,LEFT,RIGHT};
int wheelSpeed[] = {128, 128, 128, 128};
elapsedMillis sinceCompassRead;
//end Global Variables

//Motor Controller Constants
static const uint8_t FRONT_MD25_ADDR = 0xB0 >> 1;                    // Address of front MD25 motor control board
static const uint8_t REAR_MD25_ADDR = 0xB4 >> 1;                     // Address of rear MD25 motor control board
static const uint8_t PRIMARY_MD25_ADDR = 0xB0 >> 1;                    // Address of primary MD25 motor control board
static const uint8_t SECONDARY_MD25_ADDR = 0xB4 >> 1;                     // Address of secondary MD25 motor control board
    //MD25 Motor Driver READ/WRITE registers    
    static const uint8_t MD25_RIGHT_SPEED_REG = 1;    //Used in mode 0 and 1
    static const uint8_t MD25_LEFT_SPEED_REG = 0;   //Used in mode 0 and 1
    static const uint8_t MD25_FRONT_SPEED_REG = 0;    //Used in mode 0 and 1
    static const uint8_t MD25_REAR_SPEED_REG = 1;   //Used in mode 0 and 1
    static const uint8_t MD25_THROTTLE_REG = 0;        //Used in mode 2 and 3 only
    static const uint8_t MD25_TURN_REG = 1;        //Used in mode 2 and 3 only
    static const uint8_t MD25_SPEED1_REG = 0;    //Alternative name for right speed reg
    static const uint8_t MD25_SPEED2_REG = 1;    //Alternative name for left speed reg
    static const uint8_t MD25_ACCELERATION_REG = 14;	 //Acceleration rate	 Optional Acceleration register 
    static const uint8_t MD25_MODE_REG = 15;  	    //Mode	 Mode of operation (see below)
    /*
    The mode register selects which mode of operation and I2C data input type the user requires. The options being:
    0,    (Default Setting) If a value of 0 is written to the mode register then the meaning of the speed registers is literal speeds in the range of 0 (Full Reverse)  128 (Stop)   255 (Full Forward).

    1,    Mode 1 is similar to Mode 0, except that the speed registers are interpreted as signed values. The meaning of the speed registers is literal speeds in the range of -128 (Full Reverse)   0 (Stop)   127 (Full Forward).

    2,    Writing a value of  2 to the mode register will make speed1 control both motors speed, and speed2 becomes the turn value. 
    Data is in the range of 0 (Full Reverse)  128 (Stop)  255 (Full  Forward).

    3,    Mode 3 is similar to Mode 2, except that the speed registers are interpreted as signed values. 
    Data is in the range of -128  (Full Reverse)  0 (Stop)   127 (Full Forward)
    */
    static const uint8_t MD25_CMD_REG = 16;	 //Command	 Used for reset of encoder counts and module address changes, follow with a command
     // commands
          static const uint8_t MD25_ENCODER_RESET_CMD = 32;	        //20 Hex	Resets the encoder registers to zero
          static const uint8_t MD25_DISABLE_AUTO_SPEEDREG_CMD  = 48;	//30 Hex	Disables automatic speed regulation
          static const uint8_t MD25_ENABLE_AUTO_SPEEDREG_CMD  = 49;	//31 Hex	Enables automatic speed regulation (default)
          static const uint8_t MD25_DISABLE_TIMEOUT_CMD = 50;       	//32 Hex	Disables 2 second timeout of motors (Version 2 onwards only)
          static const uint8_t MD25_ENABLE_TIMEOUT_CMD = 51;        	//33 Hex	Enables 2 second timeout of motors when no I2C comms (default) (Version 2 onwards only)
          static const uint8_t MD25_ADDRESS_CHANGE_1_CMD = 160;	//A0 Hex	1st in sequence to change I2C address
          static const uint8_t MD25_ADDRESS_CHANGE_2_CMD = 170;	//AA Hex	2nd in sequence to change I2C address
          static const uint8_t MD25_ADDRESS_CHANGE_3_CMD = 165;	//A5 Hex	3rd in sequence to change I2C address
           
         
 //MD25  Motor Driver READ ONLY registers
  static const uint8_t MD25_LEFT_ENCODER_A_REG = 2;	 //Encoder 1 position, 1st byte (highest), capture count when read
  static const uint8_t MD25_LEFT_ENCODER_B_REG = 3;    	 //Encoder 1 position, 2nd byte
  static const uint8_t MD25_LEFT_ENCODER_C_REG = 4;	 //Encoder 1 position, 3rd byte
  static const uint8_t MD25_LEFT_ENCODER_D_REG = 5;	 //Encoder 1 position, 4th (lowest byte)
  
  static const uint8_t MD25_RIGHT_ENCODER_A_REG = 6;	 //Ecoder 2 position, 1st  byte (highest), capture count when read
  static const uint8_t MD25_RIGHT_ENCODER_B_REG = 7;	 //Encoder 2 position, 2nd byte
  static const uint8_t MD25_RIGHT_ENCODER_C_REG = 8;	 //Encoder 2 position, 3rd byte
  static const uint8_t MD25_RIGHT_ENCODER_D_REG = 9;	 //Encoder 2 position, 4th byte (lowest byte)
  
  static const uint8_t MD25_BATTERY_VOLTS_REG = 10;	//The supply battery voltage
  static const uint8_t MD25_FRONT_MOTOR_CURRENT_REG = 11;	//The current through motor 1
  static const uint8_t MD25_REAR_MOTOR_CURRENT_REG = 12;	//The current through motor 2
  static const uint8_t MD25_LEFT_MOTOR_CURRENT_REG = 11;	//The current through motor 1
  static const uint8_t MD25_RIGHT_MOTOR_CURRENT_REG = 12;	//The current through motor 2
  static const uint8_t MD25_VERSION_REG = 13;                   //Software Revision Number
 //end Motor Controller Constants
 
 // Compass Constants
 static const uint8_t CMPS10_ADDR = 0xC0 >> 1; // 0x60   96                                       // Defines address of CMPS10 compass
    //Compass registers, read only
    static const uint8_t CMPS10_VERSION_REG = 0;	    //Software version
    static const uint8_t CMPS10_BEARING_BYTE_REG = 1;  	    //Compass Bearing as a byte, i.e. 0-255 for a full circle
    static const uint8_t CMPS10_BEARING_DEGREE_HIGH_REG = 2;  	//Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees.
    static const uint8_t CMPS10_BEARING_DEGREE_LOW_REG = 3;  	//Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees.
    static const uint8_t CMPS10_PITCH_REG = 4;   	    //Pitch angle - signed byte giving angle in degrees from the horizontal plane
    static const uint8_t CMPS10_ROLL_REG = 5; 	            //Roll angle - signed byte giving angle in degrees from the horizontal plane
   // 6	Unused
   // 7	Unused
   // 8	Unused
   // 9	Unused
    static const uint8_t CMPS10_MAG_X_RAW_HIGH_REG = 10;    //Magnetometer X axis raw output, 16 bit signed integer with register 10 being the upper 8 bits
    static const uint8_t CMPS10_MAG_X_RAW_LOW_REG = 11;     //Magnetometer X axis raw output, 16 bit signed integer with register 11 being the lower 8 bits
    static const uint8_t CMPS10_MAG_Y_RAW_HIGH_REG = 12;    //Magnetometer Y axis raw output, 16 bit signed integer with register 12 being the upper 8 bits
    static const uint8_t CMPS10_MAG_Y_RAW_LOW_REG = 13;	    //Magnetometer Y axis raw output, 16 bit signed integer with register 13 being the lower 8 bits
    static const uint8_t CMPS10_MAG_Z_RAW_HIGH_REG = 14;    //Magnetometer Z axis raw output, 16 bit signed integer with register 14 being the upper 8 bits
    static const uint8_t CMPS10_MAG_Z_RAW_LOW_REG = 15;	    //Magnetometer Z axis raw output, 16 bit signed integer with register 15 being the lower 8 bits
    static const uint8_t CMPS10_ACCEL_X_RAW_HIGH_REG = 16;	//Accelerometer  X axis raw output, 16 bit signed integer with register 16 being the upper 8 bits
    static const uint8_t CMPS10_ACCEL_X_RAW_LOW_REG = 17;	//Accelerometer  X axis raw output, 16 bit signed integer with register 17 being the lower 8 bits
    static const uint8_t CMPS10_ACCEL_Y_RAW_HIGH_REG =  18;	//Accelerometer  Y axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
    static const uint8_t CMPS10_ACCEL_Y_RAW_LOW_REG =  19;	//Accelerometer  Y axis raw output, 16 bit signed integer with register 19 being the lower 8 bits
    static const uint8_t CMPS10_ACCEL_Z_RAW_HIGH_REG =  20;	//Accelerometer  Z axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
    static const uint8_t CMPS10_ACCEL_Z_RAW_LOW_REG =  21;	//Accelerometer  Z axis raw output, 16 bit signed integer with register 21 being the lower 8 bits   
    static const uint8_t CMPS10_CMD_REG = 22;    //Command register

 //end Compass Constants
 
 //Global Constants
 //Coordinate notations used
 // I(X,Y,ROT)  Input coordinate system refers to a 3 axis joystick. origin at center/stop position of stick.
 // C(X,Y,LEVEL) Course coordinate System(Aerial view).  Origin at lower left corner of start square.
 // typical 3 axis input
 // assumes that center/stop position lies at I( (X_IN_HIGH - X_IN_LOW /2), (Y_IN_HIGH - Y_IN_LOW /2), (ROT_IN_HIGH - ROT_IN_LOW /2) )
// note: it is OK if HIGH value is actually a lower number than LOW value.  These lables were chosen to agree with the map() parameter lables.
// adjust these values to suit the actual hardware used
static const int X_IN_LOW = 1000; // Extreme Left value
static const int X_IN_HIGH = 2000;// Extreme Right value
static const int Y_IN_LOW = 2000; // Extreme Rearward value
static const int Y_IN_HIGH = 1000;// Extreme Foreward value
static const int ROT_IN_LOW = 1000; // Extreme ClockWise value
static const int ROT_IN_HIGH = 2000;// Extreme CounterClockWise value
static const int TOLERANCE_IN = 25;


// atypical alternate 3*2 axis input(unused)
// each axis has a positive unsigned int value and bool direction.  center/stop lies at (0,0,0)
//end Global Constants

//Pin Assignments
static const uint8_t LED_PIN = 6;
static const uint8_t ICP_PIN = 4;
static const uint8_t X_IN_PIN = A1;
static const uint8_t Y_IN_PIN = A2;
static const uint8_t ROT_IN_PIN = A3;
static const uint8_t COURSE_THETA_PIN = A4;
static const uint8_t COURSE_MAGNITUDE_PIN = A5;
static const uint8_t ARM_SERVO_PIN = 26;
static const uint8_t GRIPPER_SERVO_PIN = 25;
//end Pin Assignments

// Function Prototypes
long msToAccelerate(int8_t currentSpeed, int8_t newSpeed){return 25 * abs(currentSpeed - newSpeed) / ACCELERATION;}
float countsToInches(int32_t counts) {return counts * ((WHEEL_DIA * PI)/(float)COUNTS_PER_REV);}
int32_t inchesToCounts(float inches) {return inches * COUNTS_PER_REV/(WHEEL_DIA * PI);}
int8_t getMD25SpeedByteFormat(uint8_t MD25address);
void stop();
void fullStop();
void setAutoSpeedRegulationOn(uint8_t MD25address);
void setAutoSpeedRegulationOff(uint8_t MD25address);
void setTimeoutOn(uint8_t MD25address);
void setTimeoutOff(uint8_t MD25address);
uint8_t getWheelCurrent(Wheel_t wheel);
uint8_t getVolts(uint8_t MD25address);
void setMD25SpeedByteFormat(uint8_t MD25address, uint8_t mode);
int16_t getWheelSpeed(Wheel_t wheel);
void setWheelSpeed(Wheel_t wheel, int16_t speed);// safest, slightly slower response due to mode checking.  does not assume any SpeedByteFormat or valid data, checks everything.
void drive4wheelSpeeds(int16_t speedArray[]);
void driveWheelSpeed(Wheel_t wheel, int16_t speed);// fast, potentially unsafe.  does not validate data.
void drive4wheelSpeeds(int16_t FRONTspeed, int16_t REARspeed, int16_t LEFTspeed, int16_t RIGHTspeed);
uint8_t getVersion(uint8_t device7bitAddress, uint8_t deviceVersionRegister);
void changeI2Caddress(uint8_t oldAddress, uint8_t newAddress, uint8_t commandRegister, uint8_t commandDelay); //must be run with ONLY the device whose address is to be changed connected to the I2C bus
uint8_t getAcceleration(uint8_t MD25address);
void setAcceleration(uint8_t MD25address, uint8_t value);// value from 1-10.  1 = slowest acceleration, 10 = fastest acceleration
void setAccelerationBoth(uint8_t value);// value from 1-10.  1 = slowest acceleration, 10 = fastest acceleration
void encoderReset(uint8_t MD25address);
void encoderResetBoth();
long encoder1(uint8_t MD25address);
long encoder2(uint8_t MD25address);
long getFrontEnc();
long getRearEnc();
long getLeftEnc();
long getRightEnc();
void identifyWheels();
void octagon();
void octagonEncoderInches(float inches); // drive x inches in each direction, rotate 90 degrees CCW then CW, If there is no wheel slip, robot should end in the same position it starts.
void openGrip();
void raiseArm();
void lowerArm();
void closeGrip();
void moveJoystick(int XlowRange, int XhighRange, int X, int YlowRange, int YhighRange, int Y, int RotateLowRange, int RotateHighRange, int Rotate, int Deadband);
void moveJoystick(int X, int Y, int Rotate);
void moveForward(int speed, long time);
void moveRearward(int speed, long time);
void moveLeft(int speed, long time);
void moveRight(int speed, long time);
void moveLF(int speed, long time);
void moveRF(int speed, long time);
void moveLR(int speed, long time);
void moveRR(int speed, long time);
float moveForwardInches(float inches);
float moveRearwardInches(float inches);
float moveLeftInches(float inches);
float moveRightInches(float inches);
float moveLFinches(float inches);
float moveRFinches(float inches);
float moveLRinches(float inches);
float moveRRinches(float inches);
void rotateCCW(int8_t rotateSpeed, long time);// left
void rotateCW(int8_t rotateSpeed, long time);// right
void rotateCCWdegreesEncoder(int16_t degrees);// rotate left
void rotateCWdegreesEncoder(int16_t degrees);// rotate right
void rotateCCWdegreesCompass(int16_t degrees);// rotate left
void rotateCWdegreesCompass(int16_t degrees);// rotate right
void rotateToTrueHeading(uint16_t heading);// compass relative to Magnetic North at 0 degrees
void rotateToHeading(uint16_t heading);// compass relative to 0 degree orientation when start button pressed
void serialchartEncoders();
void serialchartCompass();
float getPitchRollBearing(int8_t &pitch, int8_t &roll, uint16_t &bearing, uint16_t &fine);
void zeroCompass();

// end Function Prototypes

//Function Definitions
int8_t getMD25SpeedByteFormat(uint8_t MD25address)
{
  int8_t mode = -1;
  Wire.beginTransmission(MD25address);                      // Send byte to read mode
  Wire.write(MD25_MODE_REG);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25address, (uint8_t)1);                         // Request 1 byte form MD25
  while(Wire.available() < 1);                              // Wait for byte to arrive 
  mode = Wire.read();
  return(mode);
}

void stop()
{                                           // Function to stop motors
  Wire.beginTransmission(PRIMARY_MD25_ADDR);
  Wire.write(MD25_FRONT_SPEED_REG);
  Wire.write((int8_t)0);                                           
  Wire.endTransmission();
  
  Wire.beginTransmission(PRIMARY_MD25_ADDR);
  Wire.write(MD25_REAR_SPEED_REG);
  Wire.write((int8_t)0);                                           
  Wire.endTransmission();
  
  Wire.beginTransmission(SECONDARY_MD25_ADDR);
  Wire.write(MD25_LEFT_SPEED_REG);
  Wire.write((int8_t)0);
  Wire.endTransmission();
  
  Wire.beginTransmission(SECONDARY_MD25_ADDR);
  Wire.write(MD25_RIGHT_SPEED_REG);
  Wire.write((int8_t)0);
  Wire.endTransmission();
  wheelSpeed[FRONT] = 0;
  wheelSpeed[REAR] = 0;
  wheelSpeed[LEFT] = 0;
  wheelSpeed[RIGHT] = 0;
  delay((127/ACCELERATION) * 25);  // wait for deceleration
 
}  

void fullStop()
{
  if(0 == getMD25SpeedByteFormat(PRIMARY_MD25_ADDR) || 2 == getMD25SpeedByteFormat(PRIMARY_MD25_ADDR))
    {   
        // Function to stop motors
      Wire.beginTransmission(PRIMARY_MD25_ADDR);
      Wire.write(MD25_FRONT_SPEED_REG);
      Wire.write(128);                                  
      Wire.endTransmission();
      
      Wire.beginTransmission(PRIMARY_MD25_ADDR);
      Wire.write(MD25_REAR_SPEED_REG);
      Wire.write(128);                                          
      Wire.endTransmission();
      wheelSpeed[0] = 128;
      wheelSpeed[1] = 128;
    }
  else if(1 == getMD25SpeedByteFormat(PRIMARY_MD25_ADDR) || 3 == getMD25SpeedByteFormat(PRIMARY_MD25_ADDR))  
    {  
        // Function to stop motors
      Wire.beginTransmission(PRIMARY_MD25_ADDR);
      Wire.write(MD25_FRONT_SPEED_REG);
      Wire.write((int8_t)0);                                  
      Wire.endTransmission();
      
      Wire.beginTransmission(PRIMARY_MD25_ADDR);
      Wire.write(MD25_REAR_SPEED_REG);
      Wire.write((int8_t)0);                                          
      Wire.endTransmission();
      wheelSpeed[0] = 0;
      wheelSpeed[1] = 0;
    }
    
  
  if(0 == getMD25SpeedByteFormat(SECONDARY_MD25_ADDR) || 2 == getMD25SpeedByteFormat(SECONDARY_MD25_ADDR))
    { 
      Wire.beginTransmission(SECONDARY_MD25_ADDR);
      Wire.write(MD25_LEFT_SPEED_REG);
      Wire.write(128);                                  
      Wire.endTransmission();
      
      Wire.beginTransmission(SECONDARY_MD25_ADDR);
      Wire.write(MD25_RIGHT_SPEED_REG);
      Wire.write(128);                                          
      Wire.endTransmission();
      wheelSpeed[2] = 128;
      wheelSpeed[3] = 128;
    }    
  else if(1 == getMD25SpeedByteFormat(SECONDARY_MD25_ADDR) || 3 == getMD25SpeedByteFormat(SECONDARY_MD25_ADDR))
  {
      Wire.beginTransmission(SECONDARY_MD25_ADDR);
      Wire.write(MD25_LEFT_SPEED_REG);
      Wire.write((int8_t)0);                                  
      Wire.endTransmission();
      
      Wire.beginTransmission(SECONDARY_MD25_ADDR);
      Wire.write(MD25_RIGHT_SPEED_REG);
      Wire.write((int8_t)0);                                          
      Wire.endTransmission();
      wheelSpeed[2] = 0;
      wheelSpeed[3] = 0;
    }
    
  delay((127/ACCELERATION) * 25);  // wait for deceleration 6.375s 0.65
}  
 
void setAutoSpeedRegulationOn(uint8_t MD25address)
{
     Wire.beginTransmission(MD25address);
     Wire.write(MD25_CMD_REG);
     Wire.write(MD25_ENABLE_AUTO_SPEEDREG_CMD);   
     Wire.endTransmission();     
}

void setAutoSpeedRegulationOff(uint8_t MD25address)
{
    Wire.beginTransmission(MD25address);
     Wire.write(MD25_CMD_REG);
     Wire.write(MD25_DISABLE_AUTO_SPEEDREG_CMD);   
     Wire.endTransmission();
}
  
void setTimeoutOn(uint8_t MD25address)
{
     Wire.beginTransmission(MD25address);
     Wire.write(MD25_CMD_REG);
     Wire.write(MD25_ENABLE_TIMEOUT_CMD);   
     Wire.endTransmission();    
}  

void setTimeoutOff(uint8_t MD25address)
{
    Wire.beginTransmission(MD25address);
     Wire.write(MD25_CMD_REG);
     Wire.write(MD25_DISABLE_TIMEOUT_CMD);   
     Wire.endTransmission();
}


uint8_t getWheelCurrent(Wheel_t wheel)
{
  uint8_t current = -1;
  switch (wheel) 
  {
  case FRONT:
      Wire.beginTransmission(PRIMARY_MD25_ADDR);                      // Send byte to read current
      Wire.write(MD25_FRONT_MOTOR_CURRENT_REG);
      Wire.endTransmission();
      
      Wire.requestFrom(PRIMARY_MD25_ADDR, (uint8_t)1);                         // Request 1 byte form MD25
      while(Wire.available() < 1);                              // Wait for byte to arrive 
      current = Wire.read();                        // Get byte
      
      break;
  case REAR:
      Wire.beginTransmission(PRIMARY_MD25_ADDR);                      // Send byte to read current
      Wire.write(MD25_REAR_MOTOR_CURRENT_REG);
      Wire.endTransmission();
      
      Wire.requestFrom(PRIMARY_MD25_ADDR, (uint8_t)1);                         // Request 1 byte form MD25
      while(Wire.available() < 1);                              // Wait for byte to arrive 
      current = Wire.read();                        // Get byte
      
    break;
  case LEFT:
      Wire.beginTransmission(SECONDARY_MD25_ADDR);                      // Send byte to read current
      Wire.write(MD25_LEFT_MOTOR_CURRENT_REG);
      Wire.endTransmission();
      
      Wire.requestFrom(SECONDARY_MD25_ADDR, (uint8_t)1);                         // Request 1 byte form MD25
      while(Wire.available() < 1);                              // Wait for byte to arrive 
      current = Wire.read();                        // Get byte
      
    break;
  case RIGHT:
      Wire.beginTransmission(SECONDARY_MD25_ADDR);                      // Send byte to read current
      Wire.write(MD25_RIGHT_MOTOR_CURRENT_REG);
      Wire.endTransmission();
      
      Wire.requestFrom(SECONDARY_MD25_ADDR, (uint8_t)1);                         // Request 1 byte form MD25
      while(Wire.available() < 1);                              // Wait for byte to arrive 
      current = Wire.read();                        // Get byte
      
    break;    
 }
 return(current);
}

uint8_t getVolts(uint8_t MD25address)
{                                               // Function to read and display battery volts as a single byte
  Wire.beginTransmission(MD25address);                      // Send byte to read volts
  Wire.write(MD25_BATTERY_VOLTS_REG);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25address, (uint8_t)1);                         // Request 1 byte form MD25
  while(Wire.available() < 1);                              // Wait for byte to arrive 
  uint8_t batteryVolts = Wire.read();                        // Get byte
  return(batteryVolts);
}



void setMD25SpeedByteFormat(uint8_t MD25address, uint8_t mode)
{
  fullStop();
     Wire.beginTransmission(MD25address);
     Wire.write(MD25_MODE_REG);
     Wire.write(mode);    
     Wire.endTransmission();
    /*
    The SpeedByteFormat(mode) register selects which mode of operation and I2C data input type the user requires. The options being:
    0, unsigned bytes, speed1 speed2;   (Default Setting) If a value of 0 is written to the mode register then the meaning of the speed registers is literal speeds in the range of 0 (Full Reverse)  128 (Stop)   255 (Full Forward).

    1,  signed bytes, speed1 speed2;  Mode 1 is similar to Mode 0, except that the speed registers are interpreted as signed values. The meaning of the speed registers is literal speeds in the range of -128 (Full Reverse)   0 (Stop)   127 (Full Forward).

    2,  unsigned bytes, speed steer;  Writing a value of  2 to the mode register will make speed1 control both motors speed, and speed2 becomes the turn value. 
    Data is in the range of 0 (Full Reverse)  128 (Stop)  255 (Full  Forward).

    3,  signed bytes, speed steer;  Mode 3 is similar to Mode 2, except that the speed registers are interpreted as signed values. 
    Data is in the range of -128  (Full Reverse)  0 (Stop)   127 (Full Forward)
    */
}

int16_t getWheelSpeed(Wheel_t wheel)
{ 
  int8_t wheelSpeed = 0; 
  switch (wheel) 
    {
    case FRONT:
        Wire.beginTransmission(PRIMARY_MD25_ADDR);                      // Send byte to read wheelSpeed
        Wire.write(MD25_FRONT_SPEED_REG);
        Wire.endTransmission();
        
        Wire.requestFrom(PRIMARY_MD25_ADDR, (uint8_t)1);                         // Request 1 byte form MD25
        while(Wire.available() < 1);                              // Wait for byte to arrive 
        wheelSpeed = Wire.read();                        // Get byte
        
        break;
    case REAR:
        Wire.beginTransmission(PRIMARY_MD25_ADDR);                      // Send byte to read wheelSpeed
        Wire.write(MD25_REAR_SPEED_REG);
        Wire.endTransmission();
        
        Wire.requestFrom(PRIMARY_MD25_ADDR, (uint8_t)1);                         // Request 1 byte form MD25
        while(Wire.available() < 1);                              // Wait for byte to arrive 
        wheelSpeed = Wire.read();                        // Get byte
        
      break;
    case LEFT:
        Wire.beginTransmission(SECONDARY_MD25_ADDR);                      // Send byte to read wheelSpeed
        Wire.write(MD25_LEFT_SPEED_REG);
        Wire.endTransmission();
        
        Wire.requestFrom(SECONDARY_MD25_ADDR, (uint8_t)1);                         // Request 1 byte form MD25
        while(Wire.available() < 1);                              // Wait for byte to arrive 
        wheelSpeed = Wire.read();                        // Get byte
        
      break;
    case RIGHT:
        Wire.beginTransmission(SECONDARY_MD25_ADDR);                      // Send byte to read wheelSpeed
        Wire.write(MD25_RIGHT_SPEED_REG);
        Wire.endTransmission();
        
        Wire.requestFrom(SECONDARY_MD25_ADDR, (uint8_t)1);                         // Request 1 byte form MD25
        while(Wire.available() < 1);                              // Wait for byte to arrive 
        wheelSpeed = Wire.read();                        // Get byte
        
      break;    
   }
 return(wheelSpeed);
}

void setWheelSpeed(Wheel_t wheel, int16_t speed)// safest, slightly slower response due to mode checking.  does not assume any SpeedByteFormat or valid data, checks everything.
{
    uint8_t mode;
    if(wheel == FRONT || wheel == REAR)
    {
        mode = getMD25SpeedByteFormat(PRIMARY_MD25_ADDR);
        if(( (1 == mode || 3 == mode) && (speed >= -127 && speed <= 127) )||( (0 == mode || 2 == mode) && (speed >= 0 && speed <= 255) ))//valid range
        {
            Wire.beginTransmission(PRIMARY_MD25_ADDR);
            if(wheel == FRONT)
            {
                wheelSpeed[FRONT] = speed;
                Wire.write(MD25_FRONT_SPEED_REG);
            }
            else 
            {
                wheelSpeed[REAR] = speed;
                Wire.write(MD25_REAR_SPEED_REG);
            }
            Wire.write(speed);                                  
            Wire.endTransmission();        
             
            
        }
    }
    else if(wheel == LEFT || wheel == RIGHT)
    {
        mode = getMD25SpeedByteFormat(SECONDARY_MD25_ADDR);
        if(( (1 == mode || 3 == mode) && (speed >= -127 && speed <= 127) )||( (0 == mode || 2 == mode) && (speed >= 0 && speed <= 255) ))//valid range
        {
            Wire.beginTransmission(SECONDARY_MD25_ADDR);
            if(wheel == LEFT)
            {
                wheelSpeed[LEFT] = speed;
                Wire.write(MD25_LEFT_SPEED_REG);
            }
            else 
            {
                wheelSpeed[RIGHT] = speed;
                Wire.write(MD25_RIGHT_SPEED_REG);
            }
            Wire.write(speed);                                  
            Wire.endTransmission();
        }
    }
}

void drive4wheelSpeeds(int16_t speedArray[])
{  
    // Function to DRIVE motors
  Wire.beginTransmission(PRIMARY_MD25_ADDR);
  Wire.write(MD25_FRONT_SPEED_REG);
  Wire.write(speedArray[0]);                                  
  Wire.endTransmission();
  
  Wire.beginTransmission(PRIMARY_MD25_ADDR);
  Wire.write(MD25_REAR_SPEED_REG);
  Wire.write(speedArray[1]);                                          
  Wire.endTransmission();
  
  Wire.beginTransmission(SECONDARY_MD25_ADDR);
  Wire.write(MD25_LEFT_SPEED_REG);
  Wire.write(speedArray[2]);                                  
  Wire.endTransmission();
  
  Wire.beginTransmission(SECONDARY_MD25_ADDR);
  Wire.write(MD25_RIGHT_SPEED_REG);
  Wire.write(speedArray[3]);                                          
  Wire.endTransmission();
}

void driveWheelSpeed(Wheel_t wheel, int16_t speed)// fast, potentially unsafe.  does not validate data.
{
  switch (wheel) 
    {
  case FRONT:
    wheelSpeed[FRONT] = speed;
    Wire.beginTransmission(PRIMARY_MD25_ADDR);
    Wire.write(MD25_FRONT_SPEED_REG);
    Wire.write(speed);                                  
    Wire.endTransmission();
    break;
  case REAR:
    wheelSpeed[REAR] = speed;
    Wire.beginTransmission(FRONT_MD25_ADDR);
    Wire.write(MD25_RIGHT_SPEED_REG);
    Wire.write(speed);                                  
    Wire.endTransmission();
    break;
  case LEFT:
    wheelSpeed[LEFT] = speed;
    Wire.beginTransmission(SECONDARY_MD25_ADDR);
    Wire.write(MD25_LEFT_SPEED_REG);
    Wire.write(speed);                                  
    Wire.endTransmission();    
    break;
  case RIGHT:
    wheelSpeed[RIGHT] = speed;
    Wire.beginTransmission(SECONDARY_MD25_ADDR);
    Wire.write(MD25_RIGHT_SPEED_REG);
    Wire.write(speed);                                  
    Wire.endTransmission();
    break;  
  }
}

void drive4wheelSpeeds(int16_t FRONTspeed, int16_t REARspeed, int16_t LEFTspeed, int16_t RIGHTspeed)
{  //update array
    wheelSpeed[FRONT] = FRONTspeed;
    wheelSpeed[REAR] = REARspeed;
    wheelSpeed[LEFT] = LEFTspeed;
    wheelSpeed[RIGHT] = RIGHTspeed;
    // Function to DRIVE motors
    drive4wheelSpeeds(wheelSpeed);
}


uint8_t getVersion(uint8_t device7bitAddress, uint8_t deviceVersionRegister)
{           // Function that gets the software version
  Wire.beginTransmission(device7bitAddress);               // Send byte to read software version as a single byte
  Wire.write(deviceVersionRegister);
  Wire.endTransmission();
  
  Wire.requestFrom(device7bitAddress, (uint8_t)1);                   // request 1 byte from device
  while(Wire.available() < 1);                              // Wait for it to arrive
  uint8_t version = Wire.read();                         // Read it in
  return(version);
}


void changeI2Caddress(uint8_t oldAddress, uint8_t newAddress, uint8_t commandRegister, uint8_t commandDelay) //must be run with ONLY the device whose address is to be changed connected to the I2C bus
{
  /*
  example: changeI2Caddress(0x58, 0xB4, 16, 5); //change default md25 address 
  example: changeI2Caddress(0x60, 0xC4, 22, 5); //change default cmps10 address
  example: changeI2Caddress(0x60, 0xC4, 0, 100); //change default tpa81 address
  */
     uint8_t transmissionStatus = -1;

        
     Wire.beginTransmission(oldAddress);
     Wire.write(commandRegister);
     Wire.write(0xA0);    // also tried changing this arguement to 160
     transmissionStatus = Wire.endTransmission();
        
     delay(commandDelay + 1);
     
     Wire.beginTransmission(oldAddress);
     Wire.write(commandRegister);
     Wire.write(0xAA);    // also tried changing this arguement to 170
     transmissionStatus = Wire.endTransmission();
       
     delay(commandDelay + 1);
     
     Wire.beginTransmission(oldAddress);
     Wire.write(commandRegister);
     Wire.write(0xA5);    // also tried changing this arguement to 165
     transmissionStatus = Wire.endTransmission();
     
     delay(commandDelay + 1);
     
     Wire.beginTransmission(oldAddress);
     Wire.write(commandRegister);
     Wire.write(newAddress); 
     transmissionStatus = Wire.endTransmission();
    
     delay(1000);
     
    
}


uint8_t getAcceleration(uint8_t MD25address)
{
  Wire.beginTransmission(MD25address);                      // Send byte to read acceleration
  Wire.write(MD25_ACCELERATION_REG);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25address, (uint8_t)1);                         // Request 1 byte form MD25
  while(Wire.available() < 1);                              // Wait for byte to arrive 
  uint8_t acceleration = Wire.read();                        // Get byte
  return(acceleration);
}

void setAcceleration(uint8_t MD25address, uint8_t value)// value from 1-10.  1 = slowest acceleration, 10 = fastest acceleration
{ 
/*to calculate the time (in seconds) for the acceleration to complete :

if new speed > current speed
steps = (new speed - current speed) / acceleration register

if new speed < current speed
steps = (current speed - new speed) / acceleration register

time = steps * 25ms 

For example :

Acceleration register	Time/step	Current speed	New speed	Steps	Acceleration  time
1	25ms	0	255	255	6.375s
2	25ms	127	255	64	1.6s
3	25ms	80	0	27	0.675s
5 (default)	25ms	0	255	51	1.275s
10	25ms	255	0	26	0.65s

*/

  Wire.beginTransmission(MD25address);                      // Send byte to set acceleration
  Wire.write(MD25_ACCELERATION_REG);
  Wire.write(value);
  Wire.endTransmission();
}

void setAccelerationBoth(uint8_t value)// value from 1-10.  1 = slowest acceleration, 10 = fastest acceleration
{ 
  if(value > 0 && value < 11)
  {
    Wire.beginTransmission(PRIMARY_MD25_ADDR);                      // Send byte to set acceleration
    Wire.write(MD25_ACCELERATION_REG);
    Wire.write(value);
    Wire.endTransmission();
  
    Wire.beginTransmission(SECONDARY_MD25_ADDR);                      // Send byte to set acceleration
    Wire.write(MD25_ACCELERATION_REG);
    Wire.write(value);
    Wire.endTransmission();
  }
}

void encoderReset(uint8_t MD25address)
{                                        // This function resets the encoder values to 0
  Wire.beginTransmission(MD25address);
  Wire.write(MD25_CMD_REG);
  Wire.write(MD25_ENCODER_RESET_CMD);                                         // Putting the value 0x20 to reset encoders
  Wire.endTransmission(); 
}

void encoderResetBoth()
{                                        // This function resets the encoder values to 0
  Wire.beginTransmission(PRIMARY_MD25_ADDR);
  Wire.write(MD25_CMD_REG);
  Wire.write(MD25_ENCODER_RESET_CMD);                                         // Putting the value 0x20 to reset encoders
  Wire.endTransmission(); 
  
  Wire.beginTransmission(SECONDARY_MD25_ADDR);
  Wire.write(MD25_CMD_REG);
  Wire.write(MD25_ENCODER_RESET_CMD);                                         // Putting the value 0x20 to reset encoders
  Wire.endTransmission(); 
}

long encoder1(uint8_t MD25address)
{                                                            // Function to read value of encoder 1 as a long
  Wire.beginTransmission(MD25address);                      // Send byte to get a reading from encoder 1
  Wire.write(MD25_LEFT_ENCODER_A_REG);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25address, (uint8_t)4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1  +=Wire.read();                                     // Fourth byte for encoder 1, LL
  return(poss1);
}

long encoder2(uint8_t MD25address){                                            // Function to read velue of encoder 2 as a long
  Wire.beginTransmission(MD25address);           
  Wire.write(MD25_RIGHT_ENCODER_A_REG);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25address, (uint8_t)4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to become available
  long poss2 = Wire.read();
  poss2 <<= 8;
  poss2 += Wire.read();                
  poss2 <<= 8;
  poss2 += Wire.read();                
  poss2 <<= 8;
  poss2  +=Wire.read();      
  return(poss2);
}

long getFrontEnc()
{
  return encoder1(PRIMARY_MD25_ADDR);
}

long getRearEnc()
{
  return encoder2(PRIMARY_MD25_ADDR);
}

long getLeftEnc()
{
  return encoder1(SECONDARY_MD25_ADDR);
}

long getRightEnc()
{
  return encoder2(SECONDARY_MD25_ADDR);
}

void identifyWheels()
{
fullStop();
delay(3000);

setWheelSpeed(FRONT, 127);
delay(3000);
fullStop();
setWheelSpeed(REAR, 127);
delay(3000);
fullStop();
setWheelSpeed(LEFT, 127);
delay(3000);
fullStop();
setWheelSpeed(RIGHT, 127);
delay(3000);
fullStop();
delay(2000);
setWheelSpeed(FRONT, -127);
delay(3000);
fullStop();
setWheelSpeed(REAR, -127);
delay(3000);
fullStop();
setWheelSpeed(LEFT, -127);
delay(3000);
fullStop();
setWheelSpeed(RIGHT, -127);
delay(3000);
fullStop();
}

void countsToAccelerateForward(int8_t speed, long &accel, long &decel)
{
  long left = 0;
  long right = 0;
  stop();
  long time = msToAccelerate(0, speed);
  encoderResetBoth();
  drive4wheelSpeeds(0, 0, speed, speed);// move forward
  delay(time);
  left = getLeftEnc();
  right = getRightEnc();
  accel = (left + right)/2;
  delay(500);// be sure we are fully up to speed
  encoderReset(SECONDARY_MD25_ADDR);
  drive4wheelSpeeds(0, 0, 0, 0);
  delay(time);
  left = getLeftEnc();
  right = getRightEnc();
  decel = (left + right)/2;
}

void octagon(long segmentTime)
{
moveForward(127, segmentTime);
stop();
moveRF(127, segmentTime);
stop();
moveRight(127, segmentTime);
stop();
moveRR(127, segmentTime);
stop();
moveRearward(127, segmentTime);
stop();
moveLR(127, segmentTime);
stop();
moveLeft(127, segmentTime);
stop();
moveLF(127, segmentTime);
stop();
rotateCCW(127, segmentTime);
stop();
rotateCCW(127, segmentTime);
stop();
}

void octagonEncoderInches(float inches) // drive x inches in each direction, rotate 90 degrees CCW then CW, If there is no wheel slip, robot should end in the same position it starts.
{
  // TODO

}

void openGrip()
{
  gripServo.write(180);
}

void raiseArm()
{
  armServo.write(180);
}

void lowerArm()
{
  armServo.write(0);
}

void closeGrip()
{
  gripServo.write(0);
}

void moveJoystick(int XlowRange, int XhighRange, int X, int YlowRange, int YhighRange, int Y, int RotateLowRange, int RotateHighRange, int ROT, int Deadband)
{
	int8_t x, y, rotate;
	int front, rear, left, right = 0;

	// normalize input to signed byte range
	if(X >= (XhighRange - XlowRange /2) - Deadband && X <= (XhighRange - XlowRange /2) + Deadband)
	{ x = 0;}
	else
	{x = map(X, XlowRange, XhighRange,-128, 127);}
	if(Y >= (YhighRange - YlowRange /2) - Deadband && Y <= (YhighRange - YlowRange /2) + Deadband)
	{ y = 0;}
	else
	{y = map(Y, YlowRange, YhighRange,-128, 127);}
	if(ROT >= (RotateHighRange - RotateLowRange /2) - Deadband && ROT <= (RotateHighRange - RotateLowRange /2) + Deadband)
	{ rotate = 0;}
	else
	{rotate = map(ROT, RotateLowRange, RotateHighRange,-128, 127);}
	// convert input to wheel velocities
	front = x-rotate;
	rear = x+rotate;
	left = y-rotate;
	right = y+rotate;

	// find wheel with greatest speed 
	int * highestSpeedPtr = &front;
	if(abs(rear) > abs(*highestSpeedPtr))
	{highestSpeedPtr = &rear;}
	if(abs(left) > abs(*highestSpeedPtr))
	{highestSpeedPtr = &left;}
	if(abs(right) > abs(*highestSpeedPtr))
	{highestSpeedPtr = &right;}
	
	if(abs(*highestSpeedPtr) > 127)// at least one wheel would need to go faster than it is capable
	{// scale all wheel speeds proportionately
	// (*highestSpeed) * scaleFactor = +/-127
	float scaleFactor = 127.0/abs(*highestSpeedPtr);
	front *= scaleFactor;
	rear *= scaleFactor;
	left *= scaleFactor;
	right *= scaleFactor;	
	}

  drive4wheelSpeeds(front, rear, left, right);
}

void moveJoystick(int X, int Y, int ROT)
{
    moveJoystick(X_IN_LOW, X_IN_HIGH, X, Y_IN_LOW, Y_IN_HIGH, Y, ROT_IN_LOW, ROT_IN_HIGH, ROT, TOLERANCE_IN);
}



void moveForward(int speed, long time)
{
  drive4wheelSpeeds(0, 0, speed, speed);
  delay(time);
  stop();
}

void moveRearward(int speed, long time)
{
  drive4wheelSpeeds(0, 0, -speed, -speed);
  delay(time);
  stop();
}

void moveLeft(int speed, long time)
{
  drive4wheelSpeeds(-speed, -speed, 0, 0);
  delay(time);
  stop();
}

void moveRight(int speed, long time)
{
  drive4wheelSpeeds(speed, speed, 0, 0);
  delay(time);
  stop();
}

void moveLF(int speed, long time)
{
  drive4wheelSpeeds(-speed, -speed, speed, speed);
  delay(time);
  stop();
}

void moveRF(int speed, long time)
{
  drive4wheelSpeeds(speed, speed, speed, speed);
  delay(time);
  stop();
}

void moveLR(int speed, long time)
{
  drive4wheelSpeeds(-speed, -speed, -speed, -speed);
  delay(time);
  stop();
}

void moveRR(int speed, long time)
{
  drive4wheelSpeeds(speed, speed, -speed, -speed);
  delay(time);
  stop();
}

float moveForwardInches(float inches) // returns +/- error
{
  stop();
  encoderReset(SECONDARY_MD25_ADDR);
  int32_t desiredCount = inchesToCounts(inches); 
  
  if(desiredCount > maxAccelCount+maxDecelCount)// able to complete move at maximum speed
  {
    drive4wheelSpeeds(0, 0, 127, 127);
    while(getLeftEnc() < (desiredCount - maxDecelCount) || getLeftEnc() < (desiredCount - maxDecelCount))
    {
      delay(1);
    }
    stop();
  }
  else if(desiredCount > midAccelCount+midDecelCount)// able to complete move at medium speed
  {
    drive4wheelSpeeds(0, 0, 64, 64);
    while(getLeftEnc() < (desiredCount  - midDecelCount)|| getLeftEnc() < (desiredCount - midDecelCount))
    {
      delay(1);
    }
    stop();
  }
  else if(desiredCount > minAccelCount+minDecelCount)// able to complete move at minimum speed
  {
    drive4wheelSpeeds(0, 0, 30, 30);
    while(getLeftEnc() < (desiredCount  - minDecelCount)|| getLeftEnc() < (desiredCount - minDecelCount))
    {
      delay(1);
    }
    stop();
  }
  else// unable to complete very short move with precision
  {}
  //report error
  return countsToInches(((getLeftEnc() + getRightEnc())/2) - desiredCount); // error
}

float moveRearwardInches(float inches)
{// TODO
  int8_t speed = 127;
  drive4wheelSpeeds(0, 0, -speed, -speed);
  delay(500);
  stop();
}

float moveLeftInches(float inches)
{// TODO
  int8_t speed = 127;
  drive4wheelSpeeds(-speed, -speed, 0, 0);
  delay(500);
  stop();
}

float moveRightInches(float inches)
{// TODO
  int8_t speed = 127;
  drive4wheelSpeeds(speed, speed, 0, 0);
  delay(500);
  stop();
}

float moveLFinches(float inches)
{// TODO
  int8_t speed = 127;
  drive4wheelSpeeds(-speed, -speed, speed, speed);
  delay(500);
  stop();
}

float moveRFinches(float inches)
{// TODO
  int8_t speed = 127;
  drive4wheelSpeeds(speed, speed, speed, speed);
  delay(500);
  stop();
}

float moveLRinches(float inches)
{// TODO
  int8_t speed = 127;
  drive4wheelSpeeds(-speed, -speed, -speed, -speed);
  delay(500);
  stop();
}

float moveRRinches(float inches)
{// TODO
  int8_t speed = 127;
  drive4wheelSpeeds(speed, speed, -speed, -speed);
  delay(500);
  stop();
}

void rotateCCW(int8_t rotateSpeed, long time)// left
{
  drive4wheelSpeeds(-rotateSpeed, rotateSpeed, -rotateSpeed, rotateSpeed);
  delay(time);
  stop(); 
}

void rotateCW(int8_t rotateSpeed, long time)// right
{
  drive4wheelSpeeds(rotateSpeed, -rotateSpeed, rotateSpeed, -rotateSpeed);
  delay(time);
  stop(); 
}

void rotateCCWdegreesEncoder(int16_t degrees)// left
{// TODO
  int8_t rotateSpeed = 127;
  
  drive4wheelSpeeds(-rotateSpeed, rotateSpeed, -rotateSpeed, rotateSpeed);
  delay(500);
  stop(); 
}

void rotateCWdegreesEncoder(int16_t degrees)// right
{// TODO
  int8_t rotateSpeed = 127;
  drive4wheelSpeeds(rotateSpeed, -rotateSpeed, rotateSpeed, -rotateSpeed);
  delay(500);
  stop();
}

void rotateCCWdegreesCompass(int16_t degrees)// left
{// TODO
  int8_t rotateSpeed = 127;
  drive4wheelSpeeds(-rotateSpeed, rotateSpeed, -rotateSpeed, rotateSpeed);
  delay(500);
  stop(); 
}

void rotateCWdegreesCompass(int16_t degrees)// right
{// TODO
  int8_t rotateSpeed = 127;
  drive4wheelSpeeds(rotateSpeed, -rotateSpeed, rotateSpeed, -rotateSpeed);
  delay(500);
  stop(); 
}

void rotateToTrueHeading(uint16_t heading)// compass relative to Magnetic North at 0 degrees
{

}

void rotateToHeading(uint16_t heading)// compass relative to 0 degree orientation when start button pressed
{

}

void serialchartEncoders()
{
  int i;
            
      for(i = -128; i<=127; i++)
      {
        stop;
        encoderResetBoth();
        uint32_t accelTime =(25 * i / ACCELERATION);
        Serial.print(i);   // 1
        Serial.print(",");
        serialchartCompass(); // 2-4
        Serial.print(",");
        Serial.print(accelTime);// 5
        Serial.print(",");
        drive4wheelSpeeds(i, -i, i, -i);
        delay(accelTime);        
        Serial.print(getFrontEnc()); // 6
        Serial.print(",");
        Serial.print(getRearEnc()); // 7
        Serial.print(",");
        Serial.print(getLeftEnc()); // 8
        Serial.print(",");
        Serial.print(getRightEnc()); // 9
        Serial.print(",");
        drive4wheelSpeeds(0, 0, 0, 0);
        delay(1000);
        Serial.print(getFrontEnc()); // 10
        Serial.print(",");
        Serial.print(getRearEnc()); // 11
        Serial.print(",");
        Serial.print(getLeftEnc()); // 12
        Serial.print(",");
        Serial.print(getRightEnc()); // 13
        Serial.println();
      }
}
 
int32_t distanceToSpeed[4][256];//index1 = wheel, index2 = speedStep + 128, value = distance(in clicks) required to get up to speed.  Compare the reported distance to the actual measured distance to discover wheel slip and maximum slip free acceleration.
int32_t overshoot[4][256];//distance it takes to slow down, 
int32_t degreesPerSecond[4][256];
    
void quantifySpeeds(uint8_t acceleration)// runs robot forward, then back at each possible speed, prints results
{
	bool labels = false;
    int32_t tmp1Front = 0;
    int32_t tmp1Rear = 0;
    int32_t tmp1Left = 0;
    int32_t tmp1Right = 0;
    int32_t tmp2Front = 0;
    int32_t tmp2Rear = 0;
    int32_t tmp2Left = 0;
    int32_t tmp2Right = 0;
    
    fullStop();
    uint8_t priOldAccel = getAcceleration(PRIMARY_MD25_ADDR); //save existing acceleration rate
    uint8_t secOldAccel = getAcceleration(SECONDARY_MD25_ADDR); //save existing acceleration rate
    setAccelerationBoth(acceleration);//adjust rate
    delay(6000);// be sure bot has had time to come to a complete stop
    uint8_t speedStep;
    
    for (speedStep = 127; speedStep > 0; speedStep--)// step through each possible speed setting and record the actual speed.
    {
        
        fullStop();
        encoderResetBoth();
        drive4wheelSpeeds(speedStep, -speedStep, speedStep, -speedStep);//right turn
        delay((25 * speedStep/acceleration) + 25);//ms. wait for bot to get up to speed.
        distanceToSpeed[FRONT][speedStep] = getFrontEnc();
        distanceToSpeed[REAR][speedStep] = getRearEnc();
        distanceToSpeed[LEFT][speedStep] = getLeftEnc();
        distanceToSpeed[RIGHT][speedStep] = getRightEnc();
        delay(500);// make sure a constant speed is reached
        tmp1Front = getFrontEnc();
        tmp1Rear = getRearEnc();
        tmp1Left = getLeftEnc();
        tmp1Right = getRightEnc();
        delay(1000);
        tmp2Front = getFrontEnc();
        tmp2Rear = getRearEnc();
        tmp2Left = getLeftEnc();
        tmp2Right = getRightEnc();
        degreesPerSecond[FRONT][speedStep] = tmp2Front- tmp1Front;
        degreesPerSecond[REAR][speedStep] = tmp2Rear- tmp1Rear;
        degreesPerSecond[LEFT][speedStep] = tmp2Left- tmp1Left;
        degreesPerSecond[RIGHT][speedStep] = tmp2Right- tmp1Right;
        
        drive4wheelSpeeds(0, 0, 0, 0);
        delay((25 * speedStep/acceleration) + 400);// ensure time to stop
        tmp1Front = getFrontEnc();
        tmp1Rear = getRearEnc();
        tmp1Left = getLeftEnc();
        tmp1Right = getRightEnc();
            overshoot[FRONT][speedStep] = tmp2Front - tmp1Front;
            overshoot[REAR][speedStep] = tmp2Rear-tmp1Rear;
            overshoot[LEFT][speedStep] = tmp2Left-tmp1Left;
            overshoot[RIGHT][speedStep] = tmp2Right-tmp1Right;
            
            fullStop();
        encoderResetBoth();
        drive4wheelSpeeds(-speedStep, speedStep, -speedStep, speedStep);//left turn
        delay((25 * speedStep/acceleration) + 25);//ms. wait for bot to get up to speed.
        distanceToSpeed[FRONT][speedStep+128] = getFrontEnc();
        distanceToSpeed[REAR][speedStep+128] = getRearEnc();
        distanceToSpeed[LEFT][speedStep+128] = getLeftEnc();
        distanceToSpeed[RIGHT][speedStep+128] = getRightEnc();
        delay(500);// make sure a constant speed is reached
        tmp1Front = getFrontEnc();
        tmp1Rear = getRearEnc();
        tmp1Left = getLeftEnc();
        tmp1Right = getRightEnc();
        delay(1000);
        tmp2Front = getFrontEnc();
        tmp2Rear = getRearEnc();
        tmp2Left = getLeftEnc();
        tmp2Right = getRightEnc();
        degreesPerSecond[FRONT][speedStep+128] = tmp2Front- tmp1Front;
        degreesPerSecond[REAR][speedStep+128] = tmp2Rear- tmp1Rear;
        degreesPerSecond[LEFT][speedStep+128] = tmp2Left- tmp1Left;
        degreesPerSecond[RIGHT][speedStep+128] = tmp2Right- tmp1Right;
        
        drive4wheelSpeeds(0, 0, 0, 0);
        delay((25 * speedStep/acceleration) + 400);// ensure time to stop
        tmp1Front = getFrontEnc();
        tmp1Rear = getRearEnc();
        tmp1Left = getLeftEnc();
        tmp1Right = getRightEnc();
        overshoot[FRONT][speedStep+128] = tmp2Front - tmp1Front;
        overshoot[REAR][speedStep+128] = tmp2Rear-tmp1Rear;
        overshoot[LEFT][speedStep+128] = tmp2Left-tmp1Left;
        overshoot[RIGHT][speedStep+128] = tmp2Right-tmp1Right;
            
       
        
    }
    
    setAcceleration(PRIMARY_MD25_ADDR, priOldAccel); //restore acceleration factor to its previous value,
    setAcceleration(SECONDARY_MD25_ADDR, secOldAccel); //restore acceleration factor to its previous value,
}


void serialchartSpeedAccel()
{
  uint8_t accelStep;
  uint8_t speedStep;
  uint8_t wheel;
    for(accelStep = 10; accelStep >= 0; accelStep--)
    {        
      quantifySpeeds(accelStep);
      for(speedStep = 127; speedStep >= 0; speedStep--)
      {
        for(wheel = 0; wheel < 4; wheel++)
        {        
  	  // data for forward direction
          Serial.print(accelStep);
          Serial.print(",");
          Serial.print(speedStep);
          Serial.print(",");
          Serial.print(wheel);
          Serial.print(","); 
          Serial.print(distanceToSpeed[wheel][speedStep]);
          Serial.print(",");
          Serial.print(overshoot[wheel][speedStep]);
          Serial.print(",");
          Serial.print(degreesPerSecond[wheel][speedStep]);
          Serial.println(); 
          // data for rearward direction
          Serial.print(accelStep);
          Serial.print(",");
          Serial.print(-speedStep);
          Serial.print(",");
          Serial.print(wheel);
          Serial.print(","); 
          Serial.print(distanceToSpeed[wheel][speedStep+128]);
          Serial.print(",");
          Serial.print(overshoot[wheel][speedStep+128]);
          Serial.print(",");
          Serial.print(degreesPerSecond[wheel][speedStep+128]);
          Serial.println(); 
        }// end wheel loop
      }// end speedStep loop
    }// end accelStep loop
}
//end Motor Control Function Definitions

//Compass Function Definitions

float getPitchRollBearing(int8_t &pitch, int8_t &roll, uint16_t &bearing, uint16_t &fine) // read at most every 14ms
{
   uint8_t highByte;
   uint8_t lowByte;                // highByte and lowByte store high and low bytes of the bearing and fine stores decimal place of bearing
   float bearingFloat = 0;
   
   Wire.beginTransmission(CMPS10_ADDR);           //starts communication with CMPS10
   Wire.write(2);                              //Sends the register we wish to start reading from
   Wire.endTransmission();

   Wire.requestFrom(CMPS10_ADDR, (uint8_t)4);              // Request 4 bytes from CMPS10
   while(Wire.available() < 4);               // Wait for bytes to become available
   highByte = Wire.read();           
   lowByte = Wire.read();            
   pitch = Wire.read();              
   roll = Wire.read();               
   
   bearing = ((highByte<<8)+lowByte)/10;      // Calculate full bearing
   fine = ((highByte<<8)+lowByte)%10;         // Calculate decimal place of bearing
   bearingFloat = bearing;
   bearingFloat += fine;
   sinceCompassRead = 0;
   return bearingFloat;
}

void getRawCompassData(int16_t magX, int16_t magY, int16_t magZ, int16_t accX, int16_t accY, int16_t accZ) // read at most every 14ms
{    
   Wire.beginTransmission(CMPS10_ADDR);           //starts communication with CMPS10
   Wire.write(10);                              //Sends the register we wish to start reading from
   Wire.endTransmission();

   Wire.requestFrom(CMPS10_ADDR, (uint8_t)12);              // Request 12 bytes from CMPS10
   while(Wire.available() < 12);               // Wait for bytes to become available
   magX = Wire.read()<<8;
   magX += Wire.read();
   magY = Wire.read()<<8;
   magY += Wire.read();
   magZ = Wire.read()<<8;
   magZ += Wire.read();
   accX = Wire.read()<<8;
   accX += Wire.read();
   accY = Wire.read()<<8;
   accY += Wire.read();
   accZ = Wire.read()<<8;
   accZ += Wire.read();
   sinceCompassRead = 0;
}

#define RadToDeg 57.295779F
#define DegToRad 0.017453292F
void processRawCompassData(float Bx, float By, float Bz, float Gx, float Gy, float Gz)
{
  //Attribution Pedley  Circuit Cellar 265 (August 2012) 
  
  float Psi, Theta, Phi; // yaw, pitch, roll angles in deg
  float Vx, Vy, Vz;  // hard iron calibration coefficients
  
  float sinAngle, cosAngle;
  float Bfx, Bfy, Bfz; // calibrated mag data in uT after tilt correction
  
  // subtract hard iron interference
  Bx -= Vx;
  By -= Vy;
  Bz -= Vz;
  
  // calculate roll angle(-180 deg, 180 deg)
  Phi = atan2(Gy, Gz)*RadToDeg;
  sinAngle = sin(Phi * DegToRad);
  cosAngle = cos(Phi * DegToRad);
  // derotate by roll angle Phi
  Bfy = By * cosAngle - Bz * sinAngle; //y component
  Bz = By * sinAngle + Bz * cosAngle;
  Gz = Gy * sinAngle + Gz * cosAngle;
  
  // calculate pitch angle Theta (-90 deg, 90 deg)
  Theta = atan(-Gx / Gz) * RadToDeg;
  sinAngle = sin(Theta * DegToRad);
  cosAngle = cos(Theta * DegToRad);
  // derotate by pitch angle Theta
  Bfx = Bx * cosAngle + Bz * sinAngle; //x component
  Bfz = -Bx * sinAngle + Bz * cosAngle; //z component
  
  // calculate yaw angle Psi (-180 deg, 180 deg)
  Psi = atan2(-Bfy, Bfx) * RadToDeg;
  
}

void zeroCompass()
{

}

void serialchartCompass()
{
  int8_t pitch =0;
  int8_t roll =0;
  uint16_t bearing =0;
  uint16_t fine =0;
  float bearingFloat = getPitchRollBearing(pitch, roll, bearing, fine);
  Serial.print(bearingFloat);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.println();
}
//end Compass Function Definitions

//end Function Definitions

// Actual measured RC pulsewidths
/*neutral sticks
RC channel 0: 9399
RC channel 1: 1516
RC channel 2: 1516
RC channel 3: 1493
RC channel 4: 1522
RC channel 5: 1508
RC channel 6: 1514
end neutral sticks*/

/*left up sticks
RC channel 0: 12204
RC channel 1: 1086
RC channel 2: 1089
RC channel 3: 1093
RC channel 4: 1080
RC channel 5: 961
RC channel 6: 961
end left up sticks*/

/*right down sticks
RC channel 0: 
RC channel 1: 1946
RC channel 2: 1944
RC channel 3: 1926
RC channel 4: 1932
RC channel 5: 2067
RC channel 6: 2069
end right down sticks*/
