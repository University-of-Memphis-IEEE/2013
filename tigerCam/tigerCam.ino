//#include <SD.h>
#include <HardwareSerial.h>
//#include <Wire.h>
HardwareSerial Uart = HardwareSerial();

void setup() 
{
        
	Serial.begin(57600);
        Uart.begin(57600);
        
        pinMode(6, OUTPUT);
        digitalWrite(6,HIGH);
}

void loop() 
{        
	if (Serial.available()) 
        {	
            Uart.write(Serial.read());
	}
	if (Uart.available()) 
        {		
	    Serial.write(Uart.read());
        }
}


