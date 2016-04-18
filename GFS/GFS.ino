#include <Wire.h>
#include <Servo.h>
#include "Adafruit_Sensor.h"
#include "ITG3200.h"

ITG3200 gyro = ITG3200(123);
Servo serv;

void setup() {
  serv.attach(9);
  
  Serial.begin(9600);
  Serial.println("ITG3200 Test"); Serial.println("");

  if(!gyro.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no ITG3200 detected ... Check your wiring!");
    while(1);
  }
}

int desiredVal = 90;
unsigned long lastTime = millis();
float ellaspedTime, angle = 0;

void loop() {
  ellaspedTime = (millis() - lastTime)/1000.0;
  
  sensors_event_t gyroEvent;
  gyro.getEvent(&gyroEvent);
  
//  Serial.print("X: "); Serial.print(gyroEvent.gyro.x - gyro.X_ZERO); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(gyroEvent.gyro.y - gyro.Y_ZERO); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(gyroEvent.gyro.z - gyro.Z_ZERO); Serial.print("  ");Serial.println("rad/s");
  int error = desiredVal - (gyroEvent.gyro.z - gyro.Z_ZERO);
  Serial.print("Error: ");
  Serial.print(error);
  int servVal = PID(error);
  Serial.print(", Output: ");
  Serial.println(servVal);
  serv.writeMicroseconds(map(servVal,-1,1,700,2300));
  
  lastTime = millis();
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
