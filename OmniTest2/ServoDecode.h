#ifndef ServoDecode_h
#define ServoDecode_h
//Attribution: ServoDecode library from mem at http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1228137503/all
//adapted for Arduino 1.0.1 by Dustin Maki
//Intended for Vex, but should work with any receiver giving access to the complete pulse train on a single pin
#include "Arduino.h"
// Teensy 2.0 ICP_PIN 22, Teensy++ 2.0 ICP_PIN 4

#define TICKS_PER_uS	2	    // number of timer ticks per microsecond
typedef enum {NULL_state=-1, NOT_SYNCHED_state, ACQUIRING_state, READY_state, FAILSAFE_state } decodeState_t;
String stateStrings[] = { "NOT_SYNCHED", "ACQUIRING", "READY", "in Failsafe"};
#define MAX_CHANNELS    8	   // maximum number of channels we can store, don't increase this above 8
#define MIN_IN_PULSE_WIDTH (750 * TICKS_PER_uS) //a valid pulse must be at least 750us (note clock counts in 0.5 us ticks)
#define MAX_IN_PULSE_WIDTH (2250 * TICKS_PER_uS) //a valid pulse must be less than  2250us
#define SYNC_GAP_LEN	(3000 * TICKS_PER_uS) // we assume a space at least 3000us is sync (note clock counts in 0.5 us ticks)
//#define FAILSAFE_PIN   13  // if defined, this will set the given pin high when invalid data is received
#define PULSE_START_ON_RISING_EDGE  0
#define PULSE_START_ON_FALLING_EDGE (1<<ICES1)
#define ACQUISITION_COUNT  8  // must have this many consecutive valid frames to transition to the ready state.
volatile uint8_t pulseEnd = PULSE_START_ON_RISING_EDGE ; // default value
static volatile uint16_t Pulses[ MAX_CHANNELS + 1]; // array holding channel pulses width value in microseconds
static volatile uint16_t Failsafe[MAX_CHANNELS + 1]; // array holding channel fail safe values
static volatile uint8_t Channel;	// number of channels detected so far in the frame (first channel is 1)
static volatile uint8_t NbrChannels; // the total number of channels detected in a complete frame
static volatile decodeState_t State;	   // this will be one of the following states:
static volatile uint8_t stateCount;	   // counts the number of times this state has been repeated

static void processSync()
{
// Sync was detected so reset the channel to 1 and update the system state
   Pulses[0] = ICR1 / TICKS_PER_uS;  // save the sync pulse duration for debugging
   if(State == READY_state) 
   {
	   if( Channel != NbrChannels)
           {  // if the number of channels is unstable, go into failsafe
		   State = FAILSAFE_state;
	   }
   }
   else
   {
     if(State == NOT_SYNCHED_state)
     {
	 State = ACQUIRING_state;	  // this is the first sync pulse, we need one more to fill the channel data array
	 stateCount = 0;
     }
	 else if( State == ACQUIRING_state)	
         {
	    if(++stateCount > ACQUISITION_COUNT) 
            {
		State = READY_state;	     // this is the second sync and all channel data is ok so flag that channel data is valid
	        NbrChannels = Channel;       // save the number of channels detected
	    }
         }
	 else if( State == FAILSAFE_state)	
         {
		 if(Channel == NbrChannels)
                 {  // did we get good pulses on all channels
			State = READY_state;
		 }
	 }
   }
   Channel = 0;	 // reset the channel counter
}

// required interrupt service routines
ISR(TIMER1_OVF_vect)
{
  if(State == READY_state)
  {
    State = FAILSAFE_state;  // use fail safe values if signal lost
    Channel = 0; // reset the channel count
  }
}

ISR(TIMER1_CAPT_vect)
{
  // we want to measure the time to the end of the pulse
  if( (_SFR_BYTE(TCCR1B) & (1<<ICES1)) == pulseEnd )
  {
    TCNT1 = 0;	 // reset the counter
    if(ICR1 >= SYNC_GAP_LEN)
    {   // is the space between pulses big enough to be the SYNC
	processSync();
    }
    else if(Channel < MAX_CHANNELS) 
    {  // check if its a valid channel pulse and save it
	if( (ICR1 >= MIN_IN_PULSE_WIDTH)  && (ICR1 <= MAX_IN_PULSE_WIDTH) )
        { // check for valid channel data
	  Pulses[++Channel] = ICR1 / TICKS_PER_uS;  // store pulse length as microsoeconds
	}
	else if(State == READY_state)
        {
	  State = FAILSAFE_state;  // use fail safe values if input data invalid
	  Channel = 0; // reset the channel count
	}
    }
  }
}


class ServoDecodeClass //ServoDecodeClass begin
{
  public:
    ServoDecodeClass(){}
    
    void begin()
    {
      pinMode(ICP_PIN,INPUT);
      Channel = 0;
      State = NOT_SYNCHED_state;
      TCCR1A = 0x00;	   // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM Operation disabled
      TCCR1B = 0x02;	   // 16MHz clock with prescaler means TCNT1 increments every .5 uS (cs11 bit set
      TIMSK1 = _BV(ICIE1)|_BV (TOIE1);   // enable input capture and overflow interrupts for timer 1
      for(uint8_t chan = 1; chan <=  MAX_CHANNELS; chan++)
    	  Failsafe[chan] = Pulses[chan]= 1500; // set midpoint as default values for pulse and failsafe
    }
    
    decodeState_t getState()
    {
      return State;
    }
    
    uint8_t getChanCount()
    {
      return NbrChannels;
    }
    
    void  setFailsafe(uint8_t chan, int16_t value)
    {
    // pulse width to use if invalid data, value of 0 uses last valid data
      if( (chan > 0) && (chan <=  MAX_CHANNELS)  ) 
      {
    	 Failsafe[chan] = value;
      }
    }
    void  setFailsafe()
    {
    // setFailsafe with no arguments sets failsafe for all channels to their current values
    // usefull to capture current tx settings as failsafe values
      if(State == READY_state)
        for(uint8_t chan = 1; chan <=  MAX_CHANNELS; chan++) 
        {
    	  Failsafe[chan] = Pulses[chan];
        }
    }
    
    int16_t GetChannelPulseWidth( uint8_t channel)
    {
      // this is the access function for channel data
      int16_t result = 0; // default value
      if( channel <=  MAX_CHANNELS)  
      {
         if( (State == FAILSAFE_state)&& (Failsafe[channel] > 0 ) )
    	   result = Failsafe[channel]; // return the channels failsafe value if set and State is Failsafe
    	 else if( (State == READY_state) || (State == FAILSAFE_state) )
         {
           bitClear(TIMSK1, _BV(ICIE1)|_BV (TOIE1));//disable Timer1 input capture and overflow interrupts 
    	   //cli();	 //disable interrupts
    	   result =  Pulses[channel] ;  // return the last valid pulse width for this channel
    	   //sei(); // enable interrupts
           bitSet(TIMSK1, _BV(ICIE1)|_BV (TOIE1));//enable Timer1 input capture and overflow interrupts
    	 }
      }
      return result;
    }
    
    
};//ServoDecodeClass end


// make one instance for the user
extern ServoDecodeClass ServoDecode = ServoDecodeClass();

void printRC()//used for debugging, testing, and capturing typical values for setting up constants
{
	 
	Serial.println("");
	Serial.println("");
	static int16_t leftButtonMin = 2250;
    static int16_t leftButtonMax = 750;
    static int16_t rightButtonMin = 2250;
    static int16_t rightButtonMax = 750;

	static int16_t leftVerticalMin = 2250;
    static int16_t leftVerticalMax = 750;
    static int16_t rightVerticalMin = 2250;
    static int16_t rightVerticalMax = 750;

	static int16_t leftHorizontalMin= 2250;
    static int16_t leftHorizontalMax = 750;
    static int16_t rightHorizontalMin = 2250;
    static int16_t rightHorizontalMax = 750;

	static int16_t leftVertical = 1500;
    static int16_t rightVertical = 1500;
    static int16_t leftHorizontal = 1500;
    static int16_t rightHorizontal = 1500;
    static int16_t leftButtons = 1500;
    static int16_t rightButtons = 1500;

	static int8_t leftVerticalScaled = 0;       
    static int8_t rightVerticalScaled = 0; 
    static int8_t leftHorizontalScaled = 0;       
    static int8_t rightHorizontalScaled = 0;

    int16_t pulsewidth;    
      
        // print the decoder state
      if( ServoDecode.getState()!= READY_state) 
      {
        Serial.print("The decoder is ");
        Serial.println(stateStrings[ServoDecode.getState()]);
        for ( int i =0; i <=MAX_CHANNELS; i++ ){ // print the status of the channels
    	Serial.print("Cx "); // if you see this, the decoder does not have a valid signal
    	Serial.print(i);
    	Serial.print(" invalid pulsewidth : ");
    	pulsewidth = ServoDecode.GetChannelPulseWidth(i);
    	Serial.print(pulsewidth);
    	Serial.print("  ");
        }
        Serial.println("");
      }
      else 
      {        
        // decoder is ready, print the channel pulse widths
		leftVertical = ServoDecode.GetChannelPulseWidth(3);
        rightVertical = ServoDecode.GetChannelPulseWidth(2);
        leftHorizontal = ServoDecode.GetChannelPulseWidth(4);
        rightHorizontal = ServoDecode.GetChannelPulseWidth(1);
        leftButtons = ServoDecode.GetChannelPulseWidth(5);
        rightButtons = ServoDecode.GetChannelPulseWidth(6);

	    leftVerticalScaled = constrain(map(leftVertical, 1000, 2000, -127, 127), -127, 127);       
        rightVerticalScaled = constrain(map(rightVertical, 1000, 2000, -127, 127), -127, 127); 
        leftHorizontalScaled = constrain(map(leftHorizontal, 1000, 2000, -127, 127), -127, 127);       
        rightHorizontalScaled = constrain(map(rightHorizontal, 1000, 2000, -127, 127), -127, 127);

		if(leftButtons < leftButtonMin)
        leftButtonMin = leftButtons;
        if(leftButtons > leftButtonMax)
        leftButtonMax = leftButtons;
        
        if(rightButtons < rightButtonMin)
        rightButtonMin = rightButtons;
        if(rightButtons > rightButtonMax)
        rightButtonMax = rightButtons;

		if(leftVertical < leftVerticalMin)
        leftVerticalMin = leftVertical;
        if(leftVertical > leftVerticalMax)
        leftVerticalMax = leftVertical;
        
        if(rightVertical < rightVerticalMin)
        rightVerticalMin = rightVertical;
        if(rightVertical > rightVerticalMax)
        rightVerticalMax = rightVertical;

		if(leftHorizontal < leftHorizontalMin)
        leftHorizontalMin = leftHorizontal;
        if(leftHorizontal > leftHorizontalMax)
        leftHorizontalMax = leftHorizontal;
        
        if(rightHorizontal < rightHorizontalMin)
        rightHorizontalMin = rightHorizontal;
        if(rightHorizontal > rightHorizontalMax)
        rightHorizontalMax = rightHorizontal;

		Serial.print("leftButtons: ");
        Serial.println(leftButtons);
        Serial.print("leftButtonMin: ");
        Serial.println(leftButtonMin);
        Serial.print("leftButtonMax: ");
        Serial.println(leftButtonMax);
        Serial.println("");
        Serial.print("rightButtons: ");
        Serial.println(rightButtons);
        Serial.print("rightButtonMin: ");
        Serial.println(rightButtonMin);
        Serial.print("rightButtonMax: ");
        Serial.println(rightButtonMax);
		Serial.println("");
		Serial.println("");
		Serial.print("leftVertical: ");
        Serial.println(leftVertical);
        Serial.print("leftVerticalMin: ");
        Serial.println(leftVerticalMin);
        Serial.print("leftVerticalMax: ");
        Serial.println(leftVerticalMax);
        Serial.println("");
        Serial.print("rightVertical: ");
        Serial.println(rightVertical);
        Serial.print("rightVerticalMin: ");
        Serial.println(rightVerticalMin);
        Serial.print("rightVerticalMax: ");
        Serial.println(rightVerticalMax);
        Serial.println("");
        Serial.println("");
		Serial.print("leftHorizontal: ");
        Serial.println(leftHorizontal);
        Serial.print("leftHorizontalMin: ");
        Serial.println(leftHorizontalMin);
        Serial.print("leftHorizontalMax: ");
        Serial.println(leftHorizontalMax);
        Serial.println("");
        Serial.print("rightHorizontal: ");
        Serial.println(rightHorizontal);
        Serial.print("rightHorizontalMin: ");
        Serial.println(rightHorizontalMin);
        Serial.print("rightHorizontalMax: ");
        Serial.println(rightHorizontalMax);
		Serial.println("");
		Serial.println("");
		Serial.println("-------------------------------------");	

      }
    
}    
 
void parseRCbuttons(bool& L_UP_pushed, bool& L_DWN_pushed, bool& R_UP_pushed, bool& R_DWN_pushed, Servo& L_servo, Servo& R_servo)
{
	// note: if servo is at end of range, *_pushed will be false even when the button is actually pushed
	// this behavior can be changed if desired by removing parentheses
    int servoTmp = 90;

    if(L_UP_pushed = (ServoDecode.GetChannelPulseWidth(5) < 1500-200) &&
	 (servoTmp = L_servo.read()) < 180)// L_servo commanded and not already at maximum
	{L_servo.write(servoTmp+1);}
	if(L_DWN_pushed = (ServoDecode.GetChannelPulseWidth(5) > 1500+200) &&
	 (servoTmp = L_servo.read()) > 0)// L_servo commanded and not already at minimum
	{L_servo.write(servoTmp-1);}

	if(R_UP_pushed = (ServoDecode.GetChannelPulseWidth(6) < 1500-200) &&
	 (servoTmp = R_servo.read()) < 180)// R_servo commanded and not already at maximum
	{R_servo.write(servoTmp+1);}
	if(R_DWN_pushed = (ServoDecode.GetChannelPulseWidth(6) > 1500+200) &&
	 (servoTmp = R_servo.read()) > 0)// R_servo commanded and not already at minimum
	{R_servo.write(servoTmp-1);}  
	
	//illuminate LED on any button push
	if(L_UP_pushed || L_DWN_pushed || R_UP_pushed || R_DWN_pushed)
	{digitalWrite(6, HIGH);}
	else
	{digitalWrite(6, LOW);} 
}      

#endif



