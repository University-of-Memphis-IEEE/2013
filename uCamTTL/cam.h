/**
 * Copyright 2009
 * Sean Voisen <http://gizmologi.st>
 * Beatriz da Costa <http://beatrizdacosta.net>
 *
 * Based on the .NET driver authored by Pavel Bansky <http://banksy.net>
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at 
 *
 *   http://www.apache.org/licenses/LICENSE-2.0 
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and 
 * limitations under the License. 
 
 Modified 2012 for Arduino 1.0, HardwareSerial, and teensy++ by Dustin Maki
 */

#ifndef cam_h
#define cam_h

#include "Arduino.h"

// The byte size of commands
#define CMD_SIZE 6

// Number of sync attempts to try before giving up
#define MAX_SYNC_ATTEMPTS 60

// How long to wait for serial communication responses
#define RESPONSE_DELAY 100

// How long to wait for camera to process JPEG data
#define PROCESS_DELAY 1000

// How long to wait between data packages
#define PACKAGE_DELAY 10

// The default size of data packages when retrieving
// JPEG image data
#define DEFAULT_PACKAGE_SIZE 64

// The byte offset where image data starts in a JPEG image
// data package
#define PACKAGE_DATA_START 4

// The byte offset from the end of a data package where
// JPEG image data ends
#define PACKAGE_DATA_END_OFFSET 2

// Maximum allowed errors when reading picture data
#define MAX_ERRORS 15


/**
 * Provides a driver interface to the C328R camera from COMedia Ltd.
 4d systems uCam TTL OV528 chip
 */
class HardwareSerial;
class Cam
{
  public:
    enum ColorType 
    { 
      CT_GRAYSCALE_2 = 0x01, 
      CT_GRAYSCALE_4 = 0x02, 
      CT_GRAYSCALE_8 = 0x03, 
      CT_COLOR_8 = 0x04,
      CT_COLOR_12 = 0x05, 
      CT_COLOR_16 = 0x06, 
      CT_JPEG = 0x07 
    };

    enum PreviewResolution
    {
      PR_80x60 = 0x01,
      PR_160x120 = 0x03
    };

    enum JPEGResolution
    {
      JR_80x64 = 0x01,
      JR_160x128 = 0x03,
      JR_320x240 = 0x05,
      JR_640x480 = 0x07
    };

    enum SnapshotType
    {
      ST_COMPRESSED = 0x00,
      ST_UNCOMPRESSED = 0x01
    };

    enum PictureType
    {
      PT_SNAPSHOT = 0x01,
      PT_PREVIEW = 0x02,
      PT_JPEG = 0x05
    };

    enum FrequencyType
    {
      FT_50Hz = 0x00,
      FT_60Hz = 0x01
    };

    enum BaudRate
    {
      BAUD7200 = 0xFF,
      BAUD9600 = 0xBF,
      BAUD14400 = 0x7F,
      BAUD19200 = 0x5F,
      BAUD28800 = 0x3F,
      BAUD38400 = 0x2F,
      BAUD57600 = 0x1F,
      BAUD115200 = 0x0F
    };
    
    enum SetupReturnType
    {
      SR_SYNC_FAIL = -1,
      SR_INITIAL_FAIL = -2,
      ST_COMPRESSED = 0x00,
      ST_UNCOMPRESSED = 0x01
    };
	
	Cam(HardwareSerial *Uart = NULL);
	
    boolean sync();
    boolean reset( boolean );
    boolean powerOff();
    boolean initial( ColorType, PreviewResolution, JPEGResolution );
    boolean setLightFrequency( FrequencyType );
    boolean setPackageSize( uint16_t );
    boolean snapshot( SnapshotType, uint16_t );
    boolean getJPEGPicture( PictureType, uint16_t, void (*)(uint16_t, uint16_t, uint16_t, uint8_t*) );
    boolean setBaudRate( BaudRate );
    boolean getRawPicture( PictureType, uint8_t[], uint16_t, uint16_t );

  private:
	HardwareSerial *_serialPort;
    uint16_t _packageSize;
    uint8_t _command[CMD_SIZE];
    uint8_t _receive_cmd[CMD_SIZE];
    void createCommand( const uint8_t, uint8_t, uint8_t, uint8_t, uint8_t );
    void sendCommand();
    boolean waitForResponse( uint32_t, uint8_t[], uint16_t );
    boolean waitForResponse( uint32_t );
    boolean waitForACK( uint32_t, uint8_t );
    void sendACK( const uint8_t, uint16_t );
    void sendACK( const uint8_t );
    boolean getPicture( PictureType, uint16_t, uint16_t& );
	void serial_flush(void);
	uint8_t serial_available(void);
	void serial_print(uint8_t);
	int serial_read(void);
};

#endif

