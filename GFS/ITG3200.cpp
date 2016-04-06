#include "Arduino.h"
#include <Wire.h>
#include <limits.h>

#include "ITG3200.h"

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void ITG3200::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/

/*
byte ITG3200::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif  
  Wire.endTransmission();

  return value;
}
*/

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void HMC5883::read()
{
  // Read the magnetometer
  Wire.beginTransmission((byte)ITG3200_ADDRESS_GYRO);
  Wire.write(ITG3200_REGISTER_GYRO_GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom((byte)ITG3200_ADDRESS_GYRO, (byte)6);
  
  // Wait around until enough data is available
  while (Wire.available() < 6);

  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();
  
  // Shift values to create properly formed integer (low byte first)
  _gyroData.x = (int16_t)(((int16_t)xhi << 8) | xlo);
  _gyroData.y = (int16_t)(((int16_t)yhi << 8) | ylo);
  _gyroData.z = (int16_t)(((int16_t)zhi << 8) | zlo);
  
  // ToDo: Calculate orientation
  _magData.orientation = 0.0;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
 
/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_HMC5883 class
*/
/**************************************************************************/
ITG3200::ITG3200(int32_t sensorID) {
  _sensorID = sensorID;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
 
/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool ITG3200::begin()
{
  // Enable I2C
  Wire.begin();

  // reset itg3200
  write8(ITG3200_ADDRESS_GYRO, ITG3200_REGISTER_GYRO_PWR_MGM, PWR_MGM_RESET);

  return true;
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool ITG3200::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();
  
  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_GYROSCOPE;
  event->timestamp = 0;
  event->magnetic.x = _gyroData.x;
  event->magnetic.y = _gyroData.y;
  event->magnetic.z = _gyroData.z;
  
  return true;
}

/**************************************************************************/
/*! 
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void ITG3200::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "ITG3200", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = 0;
  // sensor->max_value   = 800; // 8 gauss == 800 microTesla
  // sensor->min_value   = -800; // -8 gauss == -800 microTesla
  // sensor->resolution  = 0.2; // 2 milligauss == 0.2 microTesla
}
