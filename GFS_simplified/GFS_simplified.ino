#include <Wire.h>
#include <Servo.h>
#include <SparkFunLSM9DS1.h>

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

LSM9DS1 imu;
Servo serv;

void setup() {
  Serial.begin(115200);
  
  serv.attach(9);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin()){
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1);
  }
}

int desiredVal = 0;
unsigned long lastTime = millis();
float ellaspedTime;

void loop() {
  imu.readGyro();
  imu.readAccel();
  imu.readMag();

  int error = desiredVal - (imu.calcGyro(imu.gx));

  int servVal = PID(error);
  serv.writeMicroseconds(map(servVal,-1,1,700,2300));
  
}

float P = .01,
      I = .01,
      D = .01;

int lastError = 0;
float integral = 0;

int PID(int error){
  float currInt = .5*ellaspedTime*(error+lastError);
  integral += currInt;
  
  int pVal = (int)(P*error);
  int iVal = (int)(I*integral);
  int dVal = (int)(D*((error-lastError)/ellaspedTime));

  return pVal + iVal + dVal;
}
