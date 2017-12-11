// Libraries
#include "Wire.h"

// Defines
#define DS3231_I2C_ADDRESS 0x68
#define chargeLED 4
#define powerLED 5
#define baud 9600

// Define state machine values
enum State
{
  STOP = 0,
  READ_FORCE,
  READ_HEART,
  READ_TEMP,
  READ_ACCELEROMETER,
  READ_GPS,
  HOUSEKEEPING,
  BLUETOOTH_CHECK_IN,
};

// Fields

/*
 * Byte 0: Is data refreshed. Incomming command will set to 1. Set to 0 when used.
 * Byte 1: Run command 0/1
*/
char dataReceived[100];

/*
 * Byte 0: Used by bluetooth master. Always set to 1
 * Byte 1: Is running
 * Bytes 2 - 6: Force sensor 1 reading in Kg 
 * Bytes 7 - 11: Force sensor 2 reading in Kg
 * Bytes 12 - 14: Heart rate in BPM
 * Bytes 15 - 17: Temperature of Battery in C
 * Bytes 18 - 20: Temperature of CPU in C
 * Bytes 21 - 44: Accelerometer readings: 8 Bytes each for x y z 
 * Bytes 45 - 60: GPS readings: 8 Bytes each for lattitude and Longitude
*/
char dataToSend[100];
State state =  STOP;
char runCommand = '0'; //Bluetooth sends ASCII, so store as char


// Initialize
void setup()
{
  // Setup Bluetooth:
  Serial.begin(baud);

  // Setup I2C bus:
  Wire.begin();
}

void loop()
{
  switch (state)
  {
    case STOP:
      //delay(100); // No OP
      state = BLUETOOTH_CHECK_IN;
    break;
    
    case READ_FORCE:
    break;
    
    case READ_HEART:
    break;
    
    case READ_TEMP:
    break;
    
    case READ_ACCELEROMETER:
    break;
    
    case READ_GPS:
    break;
    
    case HOUSEKEEPING:
    break;
    
    case BLUETOOTH_CHECK_IN:
      if(Serial.available() > 0)
      {
        Serial.readBytesUntil('\0', dataReceived, 100);
        if (dataReceived[0] == '1')
        {
          Serial.print("The message was ");
          Serial.println (dataReceived);
        }
      }
      
    
    break;
    
    default:
      Serial.write("Illegal state");
    break;
  }
}

/*Region: Helper Methods*/

/*RTC*/

/*End RTC*/



/*Accelerometer*/

/*End Accelerometer*/



/*RTC*/

/*End RTC*/



