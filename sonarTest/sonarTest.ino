#define LEFT_SRF05_PIN 4
void setup() {
  // put your setup code here, to run once:
Serial.begin(19200);
}

void loop() {
  // put your main code here, to run repeatedly: 
 
  uint16_t leftSRF05distance;                          // Stores distance
  uint16_t leftSRF05duration;                          // Stores duratiuon of pulse in
  pinMode(LEFT_SRF05_PIN, OUTPUT);
  digitalWrite(LEFT_SRF05_PIN, LOW);           // Make sure pin is low before sending a short high to trigger ranging
  delayMicroseconds(2);
  digitalWrite(LEFT_SRF05_PIN, HIGH);          // Send a short 10 microsecond high burst on pin to start ranging
  delayMicroseconds(10);
  digitalWrite(LEFT_SRF05_PIN, LOW);           // Send pin low again before waiting for pulse back in
  pinMode(LEFT_SRF05_PIN, INPUT);
  leftSRF05duration = pulseIn(LEFT_SRF05_PIN, HIGH);    // Reads echo pulse in from SRF05 in micro seconds
  leftSRF05distance = leftSRF05duration/58;              // Dividing this by 58 gives us a distance in cm
  Serial.println(leftSRF05distance);
 
}
