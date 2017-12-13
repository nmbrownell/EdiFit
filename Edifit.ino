// Libraries
#include "Wire.h"

// Defines
#define DS3231_I2C_ADDRESS 0x68
#define chargeLED 4
#define powerLED 5
#define baud 9600
#define delim "="
#define mpxA 10 // Multiplex pins for uart
#define mpxA 11
#define mpxA 12

// Define state machine values
enum State
{
  STOP = 0,
  READ_FORCE,
  READ_HEART,
  READ_TEMP,
  READ_ACCELEROMETER,
  READ_GPS,
  READ_TIME,
  HOUSEKEEPING,
  BLUETOOTH_CHECK_IN,
};

// Fields

// Parameters sent from phone
char dataReceived[100];

// Parameters to be sent to phone
int isRunning = 0;
int force1 = 0;
int force2 = 0;
int heartRate = 0;
int batTemp = 0;
int cpuTemp = 0;
int accelX = 0;
int accelY = 0;
int accelZ = 0;
int gpsLat = 0;
int gpsLong = 0;
int secs = 0;
int mins = 0;
int hrs = 0;
int days = 0;
State state =  STOP;
int runCommand = '0'; //Bluetooth sends ASCII, so store as char


// Initialize
void setup()
{
  // Setup Bluetooth:
  Serial.begin(baud);
 
  // Setup I2C bus:
  Wire.begin();
  setDS3231time(0,0,0,0,0,0,0);

}

void loop()
{
  switch (state)
  {
    case STOP:
      delay(1000); // No OP
      state = BLUETOOTH_CHECK_IN;
    break;
    
    case READ_FORCE:
      state = READ_HEART;
    break;
    
    case READ_HEART:
      state = READ_TEMP;
    break;
    
    case READ_TEMP:
      state = READ_ACCELEROMETER;
    break;
    
    case READ_ACCELEROMETER:
      state = READ_GPS;
    break;
    
    case READ_GPS:
      startGPSAccess();
      delay(100);// simulate request GPS data
      gpsLat = 100;
      gpsLong = 100;
      endGPSAccess();
      state = READ_TIME;
    break;
    
    case READ_TIME:
      byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
      // retrieve data from DS3231
      readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
      
      secs = second;
      mins = minute;
      hrs = hour;
      days = dayOfMonth;
      
      
      state = HOUSEKEEPING;
    break;
    
    case HOUSEKEEPING:
      state = BLUETOOTH_CHECK_IN;
    break;
    
    case BLUETOOTH_CHECK_IN:
      if(Serial.available() > 0)
      {
        Serial.readBytesUntil('\0', dataReceived, 100);
      }
      if (dataReceived[0] == '1')
      {
        writeParams();
        state = READ_FORCE;
      }
      else if(dataReceived[0] = '0')
      {
        state = STOP;
      }
      else
      {
        state = STOP;
      }
      delay(1000);
      
    break;
    
    default:
      Serial.write("Illegal state");
      state = READ_FORCE;
    break;
  }
}

/*Region: Helper Methods*/

/*Bluetooth*/
void writeParams()
{
  Serial.print(isRunning);
  Serial.print(delim);
  Serial.print(force1);
  Serial.print(delim);
  Serial.print(force2);
  Serial.print(delim);
  Serial.print(heartRate);
  Serial.print(delim);
  Serial.print(batTemp);
  Serial.print(delim);
  Serial.print(cpuTemp);
  Serial.print(delim);
  Serial.print(accelX);
  Serial.print(delim);
  Serial.print(accelY);
  Serial.print(delim);
  Serial.print(accelZ);
  Serial.print(delim);
  Serial.print(gpsLat);
  Serial.print(delim);
  Serial.print(gpsLong);
  Serial.print(delim);
  Serial.print(days);
  Serial.print(delim);
  Serial.print(hrs);
  Serial.print(delim);
  Serial.print(mins);
  Serial.print(delim);
  Serial.print(secs);
  Serial.println("");
}
/*End Bluetooth*/


/*RTC*/

/*End RTC*/



/*Accelerometer*/

/*End Accelerometer*/


/*GPS*/
void startGPSAccess()
{
  digitalWrite(mpxA, 0);
  digitalWrite(mpxB, 0);
  digitalWrite(mpxC, 0);
}

void endGPSAccess()
{
  digitalWrite(mpxA, 1);
  digitalWrite(mpxB, 1);
  digitalWrite(mpxC, 1);
}
/*End GPS*/


/*RTC*/
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte
dayOfMonth, byte month, byte year)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}
void readDS3231time(byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}
/*End RTC*/



