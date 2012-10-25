#include <avr/pgmspace.h>
#include "Cam.h"
#include <SD.h>
#include <Wire.h>

HardwareSerial Uart = HardwareSerial();
Cam camera = Cam(&Uart);
static const uint8_t TEENSY_LED = 6;
static const uint8_t PACKAGE_SIZE = 64;
static const uint8_t BUFFER_SIZE = 64;
static const uint8_t SKIP_FRAMES = 10;
uint8_t RxBuffer[BUFFER_SIZE];

void getJPEGPicture_callback( uint16_t picSize, uint16_t packPicDataSize, uint16_t packCount, uint8_t* pack )
{//                          pictureSize, packagePictureDataSize, packageCount, package + PACKAGE_DATA_START
  // Write this picture chunk to SD
  
}

void setup() 
{ 
pinMode(TEENSY_LED, OUTPUT);

    SD.begin();// initialize SD card.  Currently unused but available
    Serial.begin(57600); // initialize USB communication w/host computer 
    Uart.begin(57600);  // initialize TTL serial communication w/camera
    Wire.begin();// initialize I2C communication 
    
    digitalWrite( TEENSY_LED, HIGH ); // indicate power on teensy
    
    
    setupSD();
    setupCam();
    
  
}


void loop() 
{
  digitalWrite(TEENSY_LED, HIGH);  // set the LED on
  //Josh, put most of your code here.
  if( !camera.getRawPicture(Cam::PT_SNAPSHOT, RxBuffer, BUFFER_SIZE, PROCESS_DELAY ));
    {
      Serial.println( "Get RAW picture failed." );
      return;
    }
  digitalWrite(TEENSY_LED, LOW);   // set the LED off
}


boolean setupSD()
{
// see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
 // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
}

int setupCam()
{
    // synch camera
    if( !camera.sync() ) 
    {
      Serial.println( "Sync failed." );
      return SR_SYNC_FAIL;// -1
    }
    else
    {
      Serial.println( "Sync successful." );
    }
    // end synch
    // initialize camera
    if( !camera.initial( Cam::CT_COLOR_8, Cam::PR_80x60, Cam::JR_640x480 ) )
    {
      Serial.println( "Initial failed." ); 
      return SR_INITIAL_FAIL;// -2
    }
    else 
    {
    Serial.println( "Initial successful." );
    }
    // end initialize
    // setPackageSize
    if( !camera.setPackageSize( PACKAGE_SIZE ) )
    {
      Serial.println( "Package size failed." ); 
      return SR_PACKAGE_SIZE_FAIL;// -3
    }
    else
    {
      Serial.println( "Package size successful." );
    }
    // end Package size
    // set external light frequency 50Hz or 60Hz for flicker compensation
    if( !camera.setLightFrequency( Cam::FT_60Hz ) )
    {
      Serial.println( "Light frequency failed." );
      return SR_LIGHT_FREQ_FAIL;// -4
    }
    else
    {
      Serial.println( "Light frequency successful." );
    }
    // end set external light frequency
    // Let camera settle, per manual
    delay(2000);
    // test snapshot, holds image in camera buffer
    if( !camera.snapshot( Cam::ST_UNCOMPRESSED, SKIP_FRAMES ) )
    {
      Serial.println( "Snapshot failed." );
      return SR_SNAPSHOT_FAIL;// -5
    }
    else
    {
      Serial.println( " successful." );
    }
    // end test snapshot
    // get RAW picture
    if( !camera.getRawPicture(Cam::PT_SNAPSHOT, RxBuffer, BUFFER_SIZE, PROCESS_DELAY ));
    {
      Serial.println( "Get RAW picture failed." );
      return SR_RAW_FAIL;// -6
    }
    // end RAW picture
    // get JPEG
    /**
 * Gets a snapshot from the camera in compressed JPEG format.
 * Use snapshot to take a picture first.
 *
 * @param pictureType The picture type. Should be either JPEG or snapshot
 * type from the PictureType enumeration.
 * @param processDelay The time wait to process the picture with the camera's
 * on board JPEG compression. Larger pictures can take up to 1 second.
 * @param callback This is a pointer reference to a function that returns void.
 * Because the Atmega's memory is limited, and its built-in EEPROM is even smaller,
 * use this callback to handle the chunks of image data as they arrive. Each time
 * a package containing image data is transmitted, this callback will be called.
 * You will probably want to write the data to external memory of some sort,
 * appending it each time.
 *
 * @return True if successful, false otherwise
 */
    if( !Cam::getJPEGPicture(Cam::PT_JPEG, PROCESS_DELAY, &getJPEGPicture_callback );
    {
      Serial.println( "Get RAW picture failed." );
      return SR_JPEG_FAIL; // -7
    }
    // end JPEG
    // save JPEG to SD
    // occurs within callback
    // end save JPEG to SD
    
  }
}
