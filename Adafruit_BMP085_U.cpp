/***************************************************************************
  This is a library for the BMP085 pressure sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603
 
  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif

#include <math.h>
#include <limits.h>

#include "Adafruit_BMP085_U.h"


// #define BMP085_USE_DATASHEET_VALS /* Set for sanity check */

/***************************************************************************
 PRIVATE METHODS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Checks if a device with the given address is responding
*/
/**************************************************************************/
static bool CheckWire( uint8_t addr ){
	
   Wire.beginTransmission(addr);
   byte error = Wire.endTransmission();
   return (error == 0);
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_BMP085_Unified::writeCommand(byte reg, byte value)
{
  Wire.beginTransmission(addr);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_BMP085_Unified::read8(byte reg, uint8_t *value)
{
  Wire.beginTransmission(addr);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)1);
  #if ARDUINO >= 100
    *value = Wire.read();
  #else
    *value = Wire.receive();
  #endif  
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/

void Adafruit_BMP085_Unified::read16(byte reg, uint16_t *value)
{
  Wire.beginTransmission(addr);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)2);
  #if ARDUINO >= 100
    *value = (Wire.read() << 8) | Wire.read();
  #else
    *value = (Wire.receive() << 8) | Wire.receive();
  #endif  
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
void Adafruit_BMP085_Unified::readS16(byte reg, int16_t *value)
{
  uint16_t i;
  read16(reg, &i);
  *value = (int16_t)i;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void Adafruit_BMP085_Unified::readCoefficients(void)
{
  #ifdef BMP085_USE_DATASHEET_VALS
    coeffs.ac1 = 408;
    coeffs.ac2 = -72;
    coeffs.ac3 = -14383;
    coeffs.ac4 = 32741;
    coeffs.ac5 = 32757;
    coeffs.ac6 = 23153;
    coeffs.b1  = 6190;
    coeffs.b2  = 4;
    coeffs.mb  = -32768;
    coeffs.mc  = -8711;
    coeffs.md  = 2868;
    mode        = 0;
  #else
    readS16(BMP085_REGISTER_CAL_AC1, &coeffs.ac1);
    readS16(BMP085_REGISTER_CAL_AC2, &coeffs.ac2);
    readS16(BMP085_REGISTER_CAL_AC3, &coeffs.ac3);
    read16(BMP085_REGISTER_CAL_AC4, &coeffs.ac4);
    read16(BMP085_REGISTER_CAL_AC5, &coeffs.ac5);
    read16(BMP085_REGISTER_CAL_AC6, &coeffs.ac6);
    readS16(BMP085_REGISTER_CAL_B1, &coeffs.b1);
    readS16(BMP085_REGISTER_CAL_B2, &coeffs.b2);
    readS16(BMP085_REGISTER_CAL_MB, &coeffs.mb);
    readS16(BMP085_REGISTER_CAL_MC, &coeffs.mc);
    readS16(BMP085_REGISTER_CAL_MD, &coeffs.md);
  #endif
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void  Adafruit_BMP085_Unified::readRawTemperature(int32_t *temperature)
{
  #ifdef BMP085_USE_DATASHEET_VALS
    *temperature = 27898;
  #else
	uint16_t t;  
    writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
    delay(5);
	read16(BMP085_REGISTER_TEMPDATA, &t);
    *temperature = t;
  #endif
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_BMP085_Unified::readRawPressure(int32_t *pressure)
{
  #ifdef BMP085_USE_DATASHEET_VALS
    *pressure = 23843;
  #else
    uint8_t  p8;
    uint16_t p16;
    int32_t  p32;
	
	writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READPRESSURECMD + (mode << 6));
	switch(mode)
    {
      case BMP085_MODE_ULTRALOWPOWER:
        delay(5);
        break;
      case BMP085_MODE_STANDARD:
        delay(8);
        break;
      case BMP085_MODE_HIGHRES:
        delay(14);
        break;
      case BMP085_MODE_ULTRAHIGHRES:
      default:
        delay(26);
        break;
    }

    read16(BMP085_REGISTER_PRESSUREDATA, &p16);
    p32 = (uint32_t)p16 << 8;
    read8(BMP085_REGISTER_PRESSUREDATA+2, &p8);
    p32 += p8;
    p32 >>= (8 - mode);
	
    *pressure = p32;
  #endif
}

/**************************************************************************/
/*!
    @brief  Compute B5 coefficient used in temperature & pressure calcs.
*/
/**************************************************************************/
int32_t Adafruit_BMP085_Unified::computeB5(int32_t ut) {
  int32_t result = 0;	
  int32_t X1 = (ut - (int32_t)coeffs.ac6) * ((int32_t)coeffs.ac5) >> 15;
  int32_t d = (X1+(int32_t)coeffs.md);
  if (d!=0){	  // check for Zero before division! 
   int32_t X2 = ((int32_t)coeffs.mc << 11) / d;
   result = X1 + X2;
  }
  return result;
}


/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
 
/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_BMP085_Unified class i2c address and sensor ID can be changed if desired
*/
/**************************************************************************/
Adafruit_BMP085_Unified::Adafruit_BMP085_Unified(int8_t fsensor_address, int32_t fsensor_ID) {
  addr = fsensor_address;
  sensorID = fsensor_ID;
  connected = false;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
 
/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/

bool Adafruit_BMP085_Unified::begin(bmp085_mode_t fmode)
{
  // Enable I2C
  // Wire.begin(D1,D2);
   
   connected = CheckWire( addr );
	
   /* Mode boundary check */
   if ((mode > BMP085_MODE_ULTRAHIGHRES) || (mode < 0))
   {
     mode = BMP085_MODE_ULTRAHIGHRES;
   }

   /* Make sure we have the right device */
   uint8_t id;
   read8(BMP085_REGISTER_CHIPID, &id);
   if(id != 0x55)
    {
     return false;
    }

   /* Set the mode indicator */
   mode = fmode; 

   /* Coefficients need to be read once */ 
   readCoefficients();
    
   return true;
}

/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
	@param pressure reference to result
	@return boolean true if success, if false nothing is written to *pressure
	
*/
/**************************************************************************/
bool Adafruit_BMP085_Unified::getPressure(float *pressure)
{
  int32_t  ut = 0, up = 0, compp = 0;
  int32_t  x1, x2, b5, b6, x3, b3, p;
  uint32_t b4, b7;
  
  if ( ! connected ){
	  begin();
  }
  if ( ! connected ) return false;
  

  /* Get the raw pressure and temperature values */
  // readRawTemperature(&ut);
  readRawPressure(&up);

  /* Temperature compensation */
  b5 = computeB5(ut);
  /* Pressure compensation */
  b6 = b5 - 4000;
  x1 = (coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (coeffs.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) coeffs.ac1) * 4 + x3) << mode) + 2) >> 2;
  x1 = (coeffs.ac3 * b6) >> 13;
  x2 = (coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (up - b3) * (50000 >> mode));
  if (b7 < 0x80000000)
  {
    p = (b7 << 1) / b4;
  }
  else
  {
    p = (b7 / b4) << 1;
  }
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;  
  compp = p + ((x1 + x2 + 3791) >> 4);
  /* Assign compensated pressure value */
  *pressure = compp;
  
   return true;
}

/**************************************************************************/
/*!
    @brief  Reads the temperatures in degrees Celsius
	@param temp reference to result
	@return boolean true if success, if false nothing is written to *temp
*/
/**************************************************************************/
bool Adafruit_BMP085_Unified::getTemperature(float *temp)
{
  int32_t UT, B5;     // following ds convention
  float t;

  if ( ! connected ){
	begin();
  }
  if ( ! connected ) return false;
  
  readRawTemperature(&UT);

  #ifdef BMP085_USE_DATASHEET_VALS
    // use datasheet numbers!
    UT = 27898;
    coeffs.ac6 = 23153;
    coeffs.ac5 = 32757;
    coeffs.mc = -8711;
    coeffs.md = 2868;
  #endif

  B5 = computeB5(UT);
  t = (B5+8) >> 4;
  t /= 10;

  *temp = t;
  
  return true;
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
	@return resulting altitude
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::pressureToAltitude(float seaLevel, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).  Note that this
    function just calls the overload of pressureToAltitude which takes
    seaLevel and atmospheric pressure--temperature is ignored.  The original
    implementation of this function was based on calculations from Wikipedia
    which are not accurate at higher altitudes.  To keep compatibility with
    old code this function remains with the same interface, but it calls the
    more accurate calculation.

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
    @param  temp          Temperature in degrees Celsius
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::pressureToAltitude(float seaLevel, float atmospheric, float temp)
{
  return pressureToAltitude(seaLevel, atmospheric);
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude 
    (in meters), and atmospheric pressure (in hPa).  

    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
	@return resulting pressure at sealevel
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::seaLevelForAltitude(float altitude, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  
  return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude 
    (in meters), and atmospheric pressure (in hPa).  Note that this
    function just calls the overload of seaLevelForAltitude which takes
    altitude and atmospheric pressure--temperature is ignored.  The original
    implementation of this function was based on calculations from Wikipedia
    which are not accurate at higher altitudes.  To keep compatibility with
    old code this function remains with the same interface, but it calls the
    more accurate calculation.

    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
    @param  temp          Temperature in degrees Celsius
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::seaLevelForAltitude(float altitude, float atmospheric, float temp)
{
  return seaLevelForAltitude(altitude, atmospheric);
}



/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void Adafruit_BMP085_Unified::getSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "BMP085", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = sensorID;
  sensor->type        = SENSOR_TYPE_PRESSURE;
  sensor->min_delay   = 0;
  sensor->max_value   = 1100.0F;               // 300..1100 hPa
  sensor->min_value   = 300.0F;
  sensor->resolution  = 0.01F;                // Datasheet states 0.01 hPa resolution
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
	@return boolean true if success, if false nothing is written to event
*/
/**************************************************************************/
bool Adafruit_BMP085_Unified::getEvent(sensors_event_t *event)
{
  float pressure_kPa;
  
  if ( ! connected ){
	begin();
  }
  if ( ! connected ) return false;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = sensorID;
  event->type      = SENSOR_TYPE_PRESSURE;
  event->timestamp = 0;
  getPressure(&pressure_kPa);
  event->pressure = pressure_kPa / 100.0F;
  
  return true;
}
