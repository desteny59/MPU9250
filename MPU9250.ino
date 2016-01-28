#include <Wire.h>
#include "MPU9250.h"
#include <EnableInterrupt.h>

#define pwm1 2
#define pwm2 3
#define pwm3 4
#define pwm4 5


uint32_t pwm1time,pwm2time,pwm3time,pwm4time;
uint16_t pwm1value,pwm2value,pwm3value,pwm4value;

MPU9250 MPU9250;
 
// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(230400);
  MPU9250.begin();
  MPU9250.Calibration();
  pinMode(pwm1,INPUT);
  pinMode(pwm2,INPUT);
  pinMode(pwm3,INPUT);
  pinMode(pwm4,INPUT);
  enableInterrupt(pwm1, pwm1mesure, CHANGE);
  enableInterrupt(pwm2, pwm2mesure, CHANGE);
  enableInterrupt(pwm3, pwm3mesure, CHANGE);
  enableInterrupt(pwm4, pwm4mesure, CHANGE);    
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
  Serial.print(MPU9250.heading);
  Serial.print(" pwm=");
  Serial.print(pwm1value);
    Serial.print(" ");
    Serial.print(pwm2value);
    Serial.print(" ");
    Serial.print(pwm3value);
    Serial.print(" ");
    Serial.println(pwm4value);
  
  //Serial.print('\r');
   
  do{
    temps2=micros();
  }
  while((temps2-temps1)<9999);
}

void pwm1mesure()
{
  uint32_t time=micros();
  if(digitalRead(pwm1) == 1)
    {
      pwm1time=time;
    }
  else
  {
    pwm1value=time-pwm1time;
  }
}

void pwm2mesure()
{
  uint32_t time=micros();
  if(digitalRead(pwm2) == 1)
    {
      pwm2time=time;
    }
  else
  {
    pwm2value=time-pwm2time;
  }  
}

void pwm3mesure()
{
  uint32_t time=micros();
  if(digitalRead(pwm3) == 1)
    {
      pwm3time=time;
    }
  else
  {
    pwm3value=time-pwm3time;
  }  
}

void pwm4mesure()
{
  uint32_t time=micros();
  if(digitalRead(pwm4) == 1)
    {
      pwm4time=time;
    }
  else
  {
    pwm4value=time-pwm4time;
  }  
}

