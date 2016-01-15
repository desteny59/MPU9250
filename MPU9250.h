/*
9250.h
Library for gyroscope and accelerometer sensor MPU-9250 on I2C with arduino

by Pierre Muller@2015
*/

// Include guard token - prevents to include header file twice
#ifndef _MPU9250_h
#define _MPU9250_h   //create token

// Include Arduino libraries
#include "Arduino.h"
#include <Wire.h>

#define    MPU9250_ADDRESS            0x68 // can be 0x69
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18


class MPU9250{
  public:
    MPU9250();  //constructor
      void  begin();
      void  GyroCalibration();
      void  AccCalibration();
      void  MagCalibration(float uncalibrated_values[3]);
      void  UpdateRawData();
      void  FilterData();
      void  IMU_Update();
      void  I2CWriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
      void  I2CRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);

      //Variables

      int16_t rawAccX;
      int16_t rawAccY;
      int16_t rawAccZ;
      int16_t rawGyroX;
      int16_t rawGyroY;
      int16_t rawGyroZ;
      int16_t rawMagX;
      int16_t rawMagY;
      int16_t rawMagZ;
      int16_t GyroXcal;
      int16_t GyroYcal;
      int16_t GyroZcal;
      float AccX;
      float AccY;
      float AccZ;
      float GyroX;
      float GyroY;
      float GyroZ;
      float MagX;
      float MagY;
      float MagZ;
      float rawMagUncalibrated[3];
      float rawMagCalibrated[3];
      float AccXFiltered;
      float AccYFiltered;
      float AccZFiltered;
      float GyroXFiltered;
      float GyroYFiltered;
      float GyroZFiltered;
      float MagXFiltered;
      float MagYFiltered;
      float MagZFiltered;
      float pitch;
      float roll;
      float yaw;
};

#endif

