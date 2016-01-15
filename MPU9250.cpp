#include "MPU9250.h"

MPU9250::MPU9250(){
  rawAccX=0;
  rawAccY=0;
  rawAccZ=0;
  rawGyroX=0;
  rawGyroY=0;
  rawGyroZ=0;
  GyroXcal=0;
  GyroYcal=0;
  GyroZcal=0;
  rawMagX=0;
  rawMagY=0;
  rawMagZ=0;
  pitch=0;
  roll=0;
  yaw=0;
}

void MPU9250::begin(){
  // reset device
  I2CWriteByte(MPU9250_ADDRESS, 107, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
  // Configure gyroscope range
  I2CWriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  // Configure accelerometers range
  I2CWriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
  // Set by pass mode for the magnetometers
  I2CWriteByte(MPU9250_ADDRESS,0x37,0x02);
  // Set Low Pass filter of Accelerometer;
  
 
  // Request first magnetometer single measurement
  I2CWriteByte(MAG_ADDRESS,0x0A,0x16);
}

void MPU9250::GyroCalibration(){
  int i;
  long XX=0,YY=0,ZZ=0;
  for(i=0;i<1000;i++){
    UpdateRawData();
    XX += rawGyroX;
    YY += rawGyroY;
    ZZ += rawGyroZ;
  }
  GyroXcal = XX /1000;
  GyroYcal = YY /1000;
  GyroZcal = ZZ /1000;
  
}

void MPU9250::AccCalibration(){
  
}

void MPU9250::MagCalibration(float uncalibrated_values[3]){

  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    {1.719, 0.015, -0.017},
    {0.031, 1.713, 0.013},
    {-0.026, 0.033, 1.571}  
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    203.728,
    31.562,
    146.451
  };

  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) rawMagCalibrated[i] = result[i];
  
}

void MPU9250::UpdateRawData(){
  uint8_t Buf[14];
  I2CRead(MPU9250_ADDRESS,0x3B,14,Buf);

  // Accelerometer
  rawAccX=(Buf[0]<<8 | Buf[1]);
  rawAccY=(Buf[2]<<8 | Buf[3]);
  rawAccZ=Buf[4]<<8 | Buf[5];
  // Convert G
  AccX=((float)rawAccX -26.5) /1.00679 / 16384;
  AccY=((float)rawAccY -216) / 1.00216/ 16384;
  AccZ=((float)rawAccZ -181.5) / 1.00871 / 16384;
  
  // Gyroscope
  rawGyroX=(Buf[8]<<8 | Buf[9])-GyroXcal;
  rawGyroY=(Buf[10]<<8 | Buf[11])-GyroYcal;
  rawGyroZ=(Buf[12]<<8 | Buf[13])-GyroZcal;
  // Convert Deg/sec.
  GyroX = (float)rawGyroX * 250 / 32768;
  GyroY = (float)rawGyroY * 250 / 32768;
  GyroZ = (float)rawGyroZ * 250 / 32768;

  // Read magnetometer data  
  I2CRead(MAG_ADDRESS,0x03,7,Buf);
   
  // Magnetometer
  rawMagUncalibrated[0]=(Buf[3]<<8 | Buf[2]);
  rawMagUncalibrated[1]=(Buf[1]<<8 | Buf[0]);
  rawMagUncalibrated[2]=-(Buf[5]<<8 | Buf[4]);

  MagCalibration(rawMagUncalibrated);
  MagX=rawMagCalibrated[0];
  MagY=rawMagCalibrated[1];
  MagZ=rawMagCalibrated[2];
  // Normalize Mag data
  //float mag_norm=sqrt((rawMagX*rawMagX)+(rawMagY*rawMagY)+(rawMagZ*rawMagZ));
  
  //MagX=(float)rawMagX/mag_norm;
 // MagY=(float)rawMagY/mag_norm;
  //MagZ=(float)rawMagZ/mag_norm;
  
  
}

void MPU9250::FilterData(){
  float AccCoef = 0.9;
  float GyroCoef = 0.9;
  float MagCoef = 0.9;
    
  AccXFiltered = (AccX *(1-AccCoef)) + (AccXFiltered * AccCoef);
  AccYFiltered = (AccY *(1-AccCoef)) + (AccYFiltered * AccCoef);
  AccZFiltered = (AccZ *(1-AccCoef)) + (AccZFiltered * AccCoef);

  GyroXFiltered = (GyroX *(1-GyroCoef)) + (GyroXFiltered * GyroCoef);
  GyroYFiltered = (GyroY *(1-GyroCoef)) + (GyroYFiltered * GyroCoef);
  GyroZFiltered = (GyroZ *(1-GyroCoef)) + (GyroZFiltered * GyroCoef);

  MagXFiltered = (MagX *(1-MagCoef)) + (MagXFiltered * MagCoef);
  MagYFiltered = (MagY *(1-MagCoef)) + (MagYFiltered * MagCoef);
  MagZFiltered = (MagZ *(1-MagCoef)) + (MagZFiltered * MagCoef);

  
}

void MPU9250::IMU_Update(){
  UpdateRawData(); // recupération des données des capteurs
  FilterData();    // filtrage des données


    //calcul des angle depuis l'accelerometre
  float ax2=AccXFiltered*AccXFiltered;
  float ay2=AccYFiltered*AccYFiltered;
  float az2=AccZFiltered*AccZFiltered;

  //angled are radian, for degree (* 180/3.14159)
  //Pitch
  pitch = atan2(AccXFiltered,sqrt(ay2+az2))/PI*180;
    
  //Roll
  roll = atan2(AccYFiltered,sqrt(ax2+az2))/PI*180;

  //Yaw
  if(GyroZFiltered>0.1 || GyroZFiltered<-0.1)
  yaw += (GyroZFiltered/100);
  //yaw=atan2((-MagYFiltered*cos(roll)+MagZFiltered*sin(roll)),(MagXFiltered*cos(pitch)+MagYFiltered*sin(pitch)*sin(roll)+MagZFiltered*sin(pitch)*cos(roll)));
 
}

// Write a byte (Data) in device (Address) at register (Register)
void MPU9250::I2CWriteByte(uint8_t Address, uint8_t Register, uint8_t Data){
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void MPU9250::I2CRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data){
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
  Data[index++]=Wire.read();
}


