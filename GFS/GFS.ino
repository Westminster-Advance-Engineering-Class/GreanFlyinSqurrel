#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "HMC5883.h"
#include "ITG3200.h"

HMC5883 mag = HMC5883(666);
ITG3200 gyro = ITG3200(123);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("HMC5883 and ITG3200 Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  if(!gyro.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no ITG3200 detected ... Check your wiring!");
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  sensors_event_t magEvent, gyroEvent; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(magEvent.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(magEvent.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(magEvent.magnetic.z); Serial.print("  ");Serial.println("uT");

  Serial.println("");

  Serial.print("X: "); Serial.print(gyroEvent.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyroEvent.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyroEvent.gyro.z); Serial.print("  ");Serial.println("rad/s");
  

  
}
