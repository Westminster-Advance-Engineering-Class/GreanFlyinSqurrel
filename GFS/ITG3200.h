//ITG3200 datasheet: https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf

#ifndef ITG3200_H
#define ITG3200_H

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ITG3200_ADDRESS_GYRO            (0xAD0 >> 1)         // 
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      ITG3200_REGISTER_GYRO_WHO_AM_I =      0X00,   //I2C address
      ITG3200_REGISTER_GYRO_SMPLRT_DIV =    0X15,   //Sample Rate Divider
      ITG3200_REGISTER_GYRO_DLPF_FS =       0X16,   //
      ITG3200_REGISTER_GYRO_INT_CFG =       0X17,
      ITG3200_REGISTER_GYRO_INT_STATUS =    0X1A,
      ITG3200_REGISTER_GYRO_TEMP_OUT_H =    0X1B,
      ITG3200_REGISTER_GYRO_TEMP_OUT_L =    0X1C,
      ITG3200_REGISTER_GYRO_GYRO_XOUT_H =   0X1D,
      ITG3200_REGISTER_GYRO_GYRO_XOUT_L =   0X1E,
      ITG3200_REGISTER_GYRO_GYRO_YOUT_H =   0X1F,
      ITG3200_REGISTER_GYRO_GYRO_YOUT_L =   0X20,
      ITG3200_REGISTER_GYRO_GYRO_ZOUT_H =   0X21,
      ITG3200_REGISTER_GYRO_GYRO_ZOUT_L =   0X22,
      ITG3200_REGISTER_GYRO_PWR_MGM =       0X3E
    } hmc5883MagRegisters_t;
/*=========================================================================*/

/*=========================================================================
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      HMC5883_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
      HMC5883_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
      HMC5883_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
      HMC5883_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
      HMC5883_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
      HMC5883_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
      HMC5883_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
    } hmc5883MagGain;  
/*=========================================================================*/

/*=========================================================================
    INTERNAL MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct hmc5883MagData_s
    {
        float x;
        float y;
        float z;
      float orientation;
    } hmc5883MagData;
/*=========================================================================*/

/*=========================================================================
    CHIP ID
    -----------------------------------------------------------------------*/
    #define HMC5883_ID                     (0b11010100)
/*=========================================================================*/


/* Unified sensor driver for the magnetometer */
class HMC5883 : public Adafruit_Sensor
{
  public:
    HMC5883(int32_t sensorID = -1);
  
    bool begin(void);
    void setMagGain(hmc5883MagGain gain);
    bool getEvent(sensors_event_t*);
    void getSensor(sensor_t*);

  private:
    hmc5883MagGain   _magGain;
    hmc5883MagData   _magData;     // Last read magnetometer data will be available here
    int32_t         _sensorID;
    
    void write8(byte address, byte reg, byte value);
    byte read8(byte address, byte reg);
    void read(void);
};

#endif
