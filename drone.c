// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// LSM303DLHC
// This code is designed to work with the LSM303DLHC_I2CS I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/products

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h> 
#include <wiringPi.h> // Include WiringPi library!

int FR = 4,
  BR = 17,
  FL = 20,
  BL = 21;
double Heading,
  Pitch,
  Roll,
  Accx,
  Accy,
  Accz,

  Magx,
  Magy,
  Magz,
  Mag_minx,
  Mag_miny,
  Mag_minz,
  Mag_maxx,
  Mag_maxy,
  Mag_maxz;

// Pin number declarations. We're using the Broadcom chip pin numbers.
const int pwmPin = 18; // PWM LED - Broadcom pin 18, P1 pin 12
const int ledPin = 23; // Regular LED - Broadcom pin 23, P1 pin 16
const int butPin = 17; // Active-low button - Broadcom pin 17, P1 pin 11
int pwmMin = 0,
  pwmMax = 255;
int pwmValue = 75; // Use this to set an LED brightness

// Maps numbers within one range (ORIGMIN - ORIGMAX) to a new range (NEWMIN - NEWMAX)
int map(int num, int origMin, int origMax, int newMin, int newMax) {
  float origRange = origMax - origMin;
  float newRange = newMax - newMin;
  float scaler = (float)(num - origMin)/(float)(origRange);
  return (int)(newMin + (scaler * newRange));
}

void RGB(int time){

  printf("Blinker is running! Press CTRL+C to quit.\n");

  // Loop (while(1)):
  int i;
  for (i = 0; i < time; i++)
    {
      int valx = map(Magx, Mag_minx, Mag_maxx, pwmMin, pwmMax);
      int valy = map(Magy, Mag_miny, Mag_maxy, pwmMin, pwmMax);
      int valz =  map(Magz, Mag_minz, Mag_maxz, pwmMin, pwmMax);
      printf("x: %d, y: %d, z: %d\n",valx,valy,valz);
      pwmWrite(FR, valx); // PWM LED at bright setting
      pwmWrite(BR, valy); // PWM LED at bright setting
      pwmWrite(BL, valz); // PWM LED at bright setting
      sleep(1);
      pwmWrite(FR, 0); // PWM LED at bright setting
      pwmWrite(BR, 0); // PWM LED at bright setting
      pwmWrite(BL, 0); // PWM LED at bright setting
    }
}


/*
  Converts values to a tilt compensated heading in degrees (0 to 360)
*/
void getTiltHeading(void)
{
  // You can use BM004_Arduino_calibrate to measure max/min magnetometer values and plug them in here.  The values
  // below are for a specific sensor and will not match yours
  
  // use calibration values to shift and scale magnetometer measurements
  Magx = (Magx-Mag_minx)/(Mag_maxx-Mag_minx)*2-1;  
  Magy = (Magy-Mag_miny)/(Mag_maxy-Mag_miny)*2-1;  
  Magz = (Magz-Mag_minz)/(Mag_maxz-Mag_minz)*2-1;  

  // Normalize acceleration measurements so they range from 0 to 1
  double accxnorm = Accx/sqrt(Accx*Accx+Accy*Accy+Accz*Accz);
  double accynorm = Accy/sqrt(Accx*Accx+Accy*Accy+Accz*Accz);
  
  // calculate pitch and roll
  Pitch = asin(-accxnorm);
  Roll = asin(accynorm/cos(Pitch));

  // tilt compensated magnetic sensor measurements
 double magxcomp = Magx*cos(Pitch)+Magz*sin(Pitch);
 double magycomp = Magx*sin(Roll)*sin(Pitch)+Magy*cos(Roll)-Magz*sin(Roll)*cos(Pitch);
  
  // arctangent of y/x converted to degrees
  Heading = 180*atan2(magycomp,magxcomp)/M_PI;

  if (Heading < 0)
    Heading +=360;

  printf("Heading=%d\n",Heading);
}

void readGyro(int file, int *acclArray){
  
  // Get I2C device, LSM303DLHC ACCELERO I2C address is 0x19(25)
  ioctl(file, I2C_SLAVE, 0x19);
  
  // Select control register1(0x20)
  // X, Y and Z-axis enable, power on mode, o/p data rate 10 Hz(0x27)
  char config[2] = {0};
  config[0] = 0x20;
  config[1] = 0x27;
  write(file, config, 2);
	
  // Select control register4(0x23)
  // Full scale +/- 2g, continuous update(0x00)
  config[0] = 0x23;
  config[1] = 0x00;
  write(file, config, 2);
  //sleep(1);

  // Read 6 bytes of data
  // lsb first
  // Read xAccl lsb data from register(0x28)
  char reg[1] = {0x28};
  write(file, reg, 1);
  char data[1] = {0};
  if(read(file, data, 1) != 1)
    {
      printf("Error : Input/output Erorr \n");
      exit(1);
    }
	
  char data_0 = data[0];

  // Read xAccl msb data from register(0x29)
  reg[0] = 0x29;
  write(file, reg, 1);
  read(file, data, 1);
  char data_1 = data[0];

  // Read yAccl lsb data from register(0x2A)
  reg[0] = 0x2A;
  write(file, reg, 1);
  read(file, data, 1);
  char data_2 = data[0];

  // Read yAccl msb data from register(0x2B)
  reg[0] = 0x2B;
  write(file, reg, 1);
  read(file, data, 1);
  char data_3 = data[0];

  // Read zAccl lsb data from register(0x2C)
  reg[0] = 0x2C;
  write(file, reg, 1);
  read(file, data, 1);
  char data_4 = data[0];

  // Read zAccl msb data from register(0x2D)
  reg[0] = 0x2D;
  write(file, reg, 1);
  read(file, data, 1);
  char data_5 = data[0];

  // Convert the data
  int xAccl = (data_1 * 256 + data_0);
  if(xAccl > 32767)
    {
      xAccl -= 65536;
    }

  int yAccl = (data_3 * 256 + data_2);
  if(yAccl > 32767)
    {
      yAccl -= 65536;
    }

  int zAccl = (data_5 * 256 + data_4);
  if(zAccl > 32767)
    {
      zAccl -= 65536;
    }
  acclArray[0] = xAccl;
  Accx = xAccl;
  acclArray[1] = yAccl;
  Accy = yAccl;
  acclArray[2] = zAccl;
  Accz = zAccl;
  
  // Output data to screen
  printf("Acceleration in X-Axis : %d \n", xAccl);
  printf("Acceleration in Y-Axis : %d \n", yAccl);
  printf("Acceleration in Z-Axis : %d \n", zAccl);
    
}
void readMag(int file, int *magArray){
 
  
     // Get I2C device, LSM303DLHC MAGNETO I2C address is 0x1E(30)
    ioctl(file, I2C_SLAVE, 0x1E);

      
  // Select control register1(0x20)
  // X, Y and Z-axis enable, power on mode, o/p data rate 10 Hz(0x27)
  char config[2] = {0};
  config[0] = 0x20;
  config[1] = 0x27;
  write(file, config, 2);
	
  // Select control register4(0x23)
  // Full scale +/- 2g, continuous update(0x00)
  config[0] = 0x23;
  config[1] = 0x00;
  write(file, config, 2);
  //sleep(1);

  // Read 6 bytes of data
  // lsb first
  // Read xAccl lsb data from register(0x28)
  char reg[1] = {0x28};
  write(file, reg, 1);
  char data[1] = {0};
    // Select MR register(0x02)
    // Continuous conversion(0x00)
    config[0] = 0x02;
    config[1] = 0x00;
    write(file, config, 2);
    // Select CRA register(0x00)
    // Data output rate = 15Hz(0x10)
    config[0] = 0x00;
    config[1] = 0x10;
    write(file, config, 2);
    // Select CRB register(0x01)
    // Set gain = +/- 1.3g(0x20)
    config[0] = 0x01;
    config[1] = 0x20;
    write(file, config, 2);
    sleep(1);

    // Read 6 bytes of data
    // msb first
    // Read xMag msb data from register(0x03)
    reg[0] = 0x03;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_0 = data[0];

    // Read xMag lsb data from register(0x04)
    reg[0] = 0x04;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_1 = data[0];

    // Read yMag msb data from register(0x05)
    reg[0] = 0x05;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_2 = data[0];

    // Read yMag lsb data from register(0x06)
    reg[0] = 0x06;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_3 = data[0];

    // Read zMag msb data from register(0x07)
    reg[0] = 0x07;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_4 = data[0];

    // Read zMag lsb data from register(0x08)
    reg[0] = 0x08;
    write(file, reg, 1);
    read(file, data, 1);
    char data1_5 = data[0];

    // Convert the data
    int xMag = (data1_0 * 256 + data1_1);
    if(xMag > 32767)
    {
    xMag -= 65536;
    }	

    int yMag = (data1_4 * 256 + data1_5) ;
    if(yMag > 32767)
    {
    yMag -= 65536;
    }

    int zMag = (data1_2 * 256 + data1_3) ;
    if(zMag > 32767)
    {
    zMag -= 65536;
    }
    int setup = 1;
    if(setup){
    if (xMag < Mag_minx)
      Mag_minx = xMag;
    else if (xMag > Mag_maxx)
      Mag_maxx = xMag;
    
    if (yMag < Mag_miny)
      Mag_miny = yMag;
    else if (yMag > Mag_maxy)
      Mag_maxy = yMag;

    if (zMag < Mag_minz)
      Mag_minz = zMag;
    else if (zMag > Mag_maxz)
      Mag_maxz = zMag;
    }
    magArray[0] = xMag;
    Magx = xMag;
    magArray[1] = yMag;
    Magy = yMag;
    magArray[2] = zMag;
    Magz = zMag;
    // Output data to screen
    printf("Magnetic field in X-Axis : %d \n", xMag);
    printf("Magnetic field in Y-Axis : %d \n", yMag);
    printf("Magnetic field in Z-Axis : %d \n", zMag);
}

int main() 
{
  // Create I2C bus
  int file;
  char *bus = "/dev/i2c-1";
  if((file = open(bus, O_RDWR)) < 0) 
    {
      printf("Failed to open the bus. \n");
      exit(1);
    }
	
  int *acclArray,
    *magArray;
    // Setup stuff:
  wiringPiSetupGpio(); // Initialize wiringPi -- using Broadcom pin numbers

  pinMode(FR, PWM_OUTPUT); // Set PWM LED as PWM output
  pinMode(BL, PWM_OUTPUT); // Set PWM LED as PWM output
  pinMode(BR, PWM_OUTPUT); // Set PWM LED as PWM output
  pinMode(BL, PWM_OUTPUT); // Set PWM LED as PWM output
  acclArray = malloc(3*sizeof(int));
  magArray = malloc(3*sizeof(int));
  // Loop
  while(1){
    //readGyro(file, acclArray);
    readMag(file, magArray);
    //RGB(10);
    //getTiltHeading();
    int r, g, b;
    for (r = 0; r < pwmMax; r++){
      for (g = 0; g < pwmMax; g++){
	for (b = 0; b < pwmMax; b++){
	  pwmWrite(FR, r); // PWM LED at bright setting
	  pwmWrite(BR, g); // PWM LED at bright setting
	  pwmWrite(BL, b); // PWM LED at bright setting
	  sleep(.02);
	  //pwmWrite(FR, 0); // PWM LED at bright setting
	  //pwmWrite(BR, 0); // PWM LED at bright setting
	  //pwmWrite(BL, 0); // PWM LED at bright setting
	}
      }
    }
  }
  return 0;
}
