// Libraries
#include "Wire.h"
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

// Defines
#define DS3231_I2C_ADDRESS 0x68
#define chargeLED 4
#define powerLED 5
#define baud 9600
#define delim " | "
#define mpxA 10 // Multiplex pins for uart
#define mpxB 11
#define mpxC 12
#define threshold 550 // for heartrate
#define heartADC 7
#define thermADC 6
#define fsrPinf 2
#define fsrPinb 3

int hrSignal = 0;
Adafruit_MMA8451 mma = Adafruit_MMA8451();

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
int steps = 0;
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
  // Setup UART:
  Serial.begin(baud);
  endGPSAccess(); // Multiplex to BT mode
 
  // Setup I2C bus:
  Wire.begin();
  setDS3231time(0,0,0,0,0,0,0);

  mma.begin();
  mma.setRange(MMA8451_RANGE_8_G);
}

void loop()
{
  switch (state)
  {
    case STOP:
      delay(500); // No OP
      setDS3231time(0,0,0,0,0,0,0);
      isRunning = 0;
      steps = 0;
      heartRate = 0;
      batTemp = 0;
      cpuTemp = 0;
      accelX = 0;
      accelY = 0;
      accelZ = 0;
      gpsLat = 0;
      gpsLong = 0;
      secs = 0;
      mins = 0;
      hrs = 0;
      days = 0;
      state = BLUETOOTH_CHECK_IN;
    break;
    
    case READ_FORCE:
      readFSR();
      state = READ_HEART;
    break;
    
    case READ_HEART:
      hrSignal = analogRead(heartADC); 
      if(hrSignal > threshold)                     
        heartRate = 1;
      else
        heartRate = 0;
      state = READ_TEMP;
    break;
    
    case READ_TEMP:
      batTemp = analogRead(thermADC); 
      state = READ_ACCELEROMETER;
    break;
    
    case READ_ACCELEROMETER:
      mma.read();
      sensors_event_t event; 
      mma.getEvent(&event);

      accelX = event.acceleration.x;
      accelY = event.acceleration.y;
      accelZ = event.acceleration.z;
      
        //readAccel();
      state = READ_GPS;
    break;
    
    case READ_GPS:
      //startGPSAccess();
      //delay(100);// simulate request GPS data
      //gpsLat = 100;
      //gpsLong = 100;
      //endGPSAccess();
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
        isRunning = 1;
        state = READ_FORCE;
      }
      else if(dataReceived[0] = '0')
      {
        isRunning = 0;
        state = STOP;
      }
      else
      {
        state = STOP;
      }
      delay(10);
      
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
  Serial.print("running: " );
  Serial.print( isRunning);
  Serial.print(delim);
  Serial.print("steps: ");
  Serial.print( steps);
  Serial.print(delim);
  Serial.print("heartrate: " );
  Serial.print( heartRate);
  Serial.print(delim);
  Serial.print("battemp: " );
  Serial.print( batTemp);
  Serial.print(delim);
  Serial.print("cputemp: " );
  Serial.print( cpuTemp);
  Serial.print(delim);
  Serial.print("accelX: " );
  Serial.print( accelX);
  Serial.print(delim);
  Serial.print("accelY: " );
  Serial.print( accelY);
  Serial.print(delim);
  Serial.print("accelZ: " );
  Serial.print( accelZ);
  Serial.print(delim);
  Serial.print("gpsLat: " );
  Serial.print( gpsLat);
  Serial.print(delim);
  Serial.print("gpsLong: " );
  Serial.print( gpsLong);
  Serial.print(delim);
  Serial.print("days: " );
  Serial.print( days);
  Serial.print(delim);
  Serial.print("hrs: " );
  Serial.print( hrs);
  Serial.print(delim);
  Serial.print("mins: " );
  Serial.print( mins);
  Serial.print(delim);
  Serial.print("secs: " );
  Serial.print( secs);
  Serial.println("");
}
/*End Bluetooth*/


/*FSR*/
int fsrCount = 0; // Remember number of times fsr triggered
void readFSR()
{
  int fsrReadingf = analogRead(fsrPinf);
  int fsrReadingb = analogRead(fsrPinb);

  //Serial.println("Analog reading front = ");
  //Serial.println(fsrReadingf);     // the raw analog reading
  //Serial.println("Analog reading back = ");
  //Serial.println(fsrReadingb);     // the raw analog reading
  if (fsrCount == 0  && (fsrReadingf < 900 || fsrReadingb < 900))
    fsrCount++;
  else if (fsrCount == 1 && (fsrReadingf > 900 || fsrReadingb > 900)) {
    fsrCount = 0;
    steps = steps + 2;
  }
}
/*End FSR*/



/*Accelerometer*/
void readAccel()
{
    // Read the 'raw' data in 14-bit counts
  mma.read();
//  Serial.print("X:\t"); 
//  Serial.print(mma.x); 
//  Serial.print("\tY:\t"); 
//  Serial.print(mma.y); 
//  Serial.print("\tZ:\t");
//  Serial.print(mma.z); 
//  Serial.println();

  /* Get a new sensor event */ 
  sensors_event_t event; 
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  //Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  //Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  //Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
  //Serial.println("m/s^2 ");
  
  /* Get the orientation of the sensor */
  uint8_t o = mma.getOrientation();

  int i = 0; /* initial step_count */
  int j = 1; /* initial sample count  */
  int Current_Average; 
  int Moving_Average; 


  int k = event.acceleration.y; /* current data value */
  //accelY = k;
  Current_Average = AveragingFunction(j,k); 
  //intensity = Current_Average;
}

int AveragingFunction(int sample_count, int current_data_value) 
{ 
  int Moving_Average;
  int j; 
  Moving_Average = (Moving_Average * sample_count + abs(current_data_value)) / (sample_count + 1); 
  sample_count++;
  j = sample_count; 
  return Moving_Average;
}
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
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
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



