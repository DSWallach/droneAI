char filename[20];
sprintf(filename, "/dev/i2c-%d", 1);
file = open(filename, O_RDWR);
if (file<0) {
  printf("Unable to open I2C bus!");
  exit(1);
 }

// Enable accelerometer.
writeAccReg(LSM303_CTRL_REG1_A, 0b01010111); //  z,y,x axis enabled , 100Hz data rate
writeAccReg(LSM303_CTRL_REG4_A, 0b00101000); // +/- 8G full scale: FS = 10 on DLHC, high resolution output mode

// Enable Gyro
writeGyrReg(L3G_CTRL_REG1, 0b00001111); // Normal power mode, all axes enabled
writeGyrReg(L3G_CTRL_REG4, 0b00110000); // Continuous update, 2000 dps full scale
void readACC(int  *a)
{
  uint8_t block[6];
  selectDevice(file,ACC_ADDRESS);

  readBlock(0x80 | LSM303_OUT_X_L_A, sizeof(block), block);

  *a = (int16_t)(block[0] | block[1] << 8) >> 4;
  *(a+1) = (int16_t)(block[2] | block[3] << 8) >> 4;
  *(a+2) = (int16_t)(block[4] | block[5] << 8) >> 4;
}

void readGYR(int *g)
{
  uint8_t block[6];
  selectDevice(file,GYR_ADDRESS);

  readBlock(0x80 | L3G_OUT_X_L, sizeof(block), block);

  *g = (int16_t)(block[1] << 8 | block[0]);
  *(g+1) = (int16_t)(block[3] << 8 | block[2]);
  *(g+2) = (int16_t)(block[5] << 8 | block[4]); }

//Convert Gyro raw to degrees per second
rate_gyr_x = (float) *gyr_raw * G_GAIN;
rate_gyr_y = (float) *(gyr_raw+1) * G_GAIN;
rate_gyr_z = (float) *(gyr_raw+2) * G_GAIN;

//Calculate the angles from the gyro
gyroXangle+=rate_gyr_x*DT;
gyroYangle+=rate_gyr_y*DT;
gyroZangle+=rate_gyr_z*DT;

//Convert Accelerometer values to degrees

AccXangle = (float) (atan2(*(acc_raw+1),*(acc_raw+2))+M_PI)*RAD_TO_DEG;
AccYangle = (float) (atan2(*(acc_raw+2),*acc_raw)+M_PI)*RAD_TO_DEG;

//Change the rotation value of the accelerometer to -/+ 180
if (AccXangle >180)
  {
    AccXangle -= (float)360.0;
  }
if (AccYangle >180)
  AccYangle -= (float)360.0;

CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;


startInt = mymillis();

//Each loop should be at least 20ms.
while(mymillis() - startInt < 20)
  {
    printf("hi");
    usleep(100);
  }

