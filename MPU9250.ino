#include <Wire.h>
#include "MPU9250.h"
//#include "MadgwickAHRS.h"




MPU9250 MPU9250;
 
// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(230400);
  MPU9250.begin();
  MPU9250.Calibration();
    
}
 
 
// Main loop, read and display data
void loop()
{
  long temps1=0,temps2=0;
  temps1=micros();
  MPU9250.IMU_Update();
  Serial.print("Pitch =");
  Serial.print(MPU9250.pitch);
  Serial.print(" Roll = ");
  Serial.print(MPU9250.roll);
  Serial.print(" Yaw = ");
  Serial.print(MPU9250.yaw);
  Serial.print(" Heading =");
  Serial.println(MPU9250.heading);
  
  //Serial.print('\r');
   
  do{
    temps2=micros();
  }
  while((temps2-temps1)<9999);
}


