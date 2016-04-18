//ITG3200 datasheet: https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf

#ifndef ITG3200_H
#define ITG3200_H

#include "Arduino.h"
#include "Adafruit_Sensor.h"
#include <Wire.h>


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ITG3200_ADDRESS_GYRO            (0x69)         // 
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
    } itg3200GyroRegisters_t;
/*=========================================================================*/

/*=========================================================================*
    SETTINGS
    -----------------------------------------------------------------------*/

    typedef enum
    {
      DLPF_CFG_0 =    0x01, //1<<0,
      DLPF_CFG_1 =    0x02, //1<<1,
      DLPF_CFG_2 =    0x04, //1<<2,
      DLPF_FS_SEL_0 = 0x08, //1<<3,
      DLPF_FS_SEL_1 = 0x10, //1<<4,
      PWR_MGM_RESET = 0x40 //1<<6
    } itg3200GyroSettings_t;
/*=========================================================================*/

/*=========================================================================
    INTERNAL MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct itg3200GyroData_s
    {
        float x;
        float y;
        float z;
    } itg3200GyroData;
/*=========================================================================*/
    

/* Unified sensor driver for the magnetometer */
class ITG3200 : public Adafruit_Sensor
{
  public:
    ITG3200(int32_t sensorID = -1);
  
    bool begin(void);
    bool getEvent(sensors_event_t*);
    void getSensor(sensor_t*);

    const static int X_ZERO = 31,
               Y_ZERO = -9,
               Z_ZERO = 26;
  
  private:
    itg3200GyroData   _gyroData;     // Last read gyroscopic sensor data will be available here
    int32_t       _sensorID;
    
    void write8(byte address, byte reg, byte value);
    //byte read8(byte address, byte reg);
    void read(void);
    int zeros[3];
};

#endif
