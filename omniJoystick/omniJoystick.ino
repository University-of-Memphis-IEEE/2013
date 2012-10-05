#include <Wire.h>
#include "configure.h"

int loopCount = 0;

void setup()
{
  Wire.begin();  
  Serial.begin(9600);
  
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  Serial.println("setup() entered.");
  
setAutoSpeedRegulationOn(PRIMARY_MD25_ADDR);
setAutoSpeedRegulationOn(SECONDARY_MD25_ADDR);
setTimeoutOff(PRIMARY_MD25_ADDR);
setTimeoutOff(SECONDARY_MD25_ADDR);
setMD25SpeedByteFormat(PRIMARY_MD25_ADDR, 1);
setMD25SpeedByteFormat(SECONDARY_MD25_ADDR, 1);

Serial.println("End of Setup()");
Serial.print("Loop:");
Serial.println(loopCount);
digitalWrite(6, LOW);
delay(2000);
}

void loop()
{
  digitalWrite(6, HIGH);
  loopCount++;  
  Serial.print("Loop:");
  Serial.println(loopCount);
  // speed  -128 (Full Left)   0 (Stop)   127 (Full Right)
  // speed  -128 (Full Reverse)   0 (Stop)   127 (Full Forward)
  // speed  -128 (Full ClockWise)   0 (Stop)   127 (Full CounterClockwise)
  
  
  moveJoystick(analogRead(X_PIN), analogRead(Y_PIN), analogRead(ROT_PIN_PIN));
  
  digitalWrite(6, LOW);
  delay(300);
}

