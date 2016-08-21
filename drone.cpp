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
/*
// Registers/etc.
short __MODE1              = 0x00,
  __MODE2              = 0x01,
  __SUBADR1            = 0x02,
  __SUBADR2            = 0x03,
  __SUBADR3            = 0x04,
  __PRESCALE           = 0xFE,
  __LED0_ON_L          = 0x06,
  __LED0_ON_H          = 0x07,
  __LED0_OFF_L         = 0x08,
  __LED0_OFF_H         = 0x09,
  __ALL_LED_ON_L       = 0xFA,
  __ALL_LED_ON_H       = 0xFB,
  __ALL_LED_OFF_L      = 0xFC,
  __ALL_LED_OFF_H      = 0xFD,

// Bits
  __RESTART            = 0x80,
  __SLEEP              = 0x10,
  __ALLCALL            = 0x01,
  __INVRT              = 0x10,
  __OUTDRV             = 0x04;

void softReset(){
}

void initPWM(int file, short address){
   ioctl(file, I2C_SLAVE, address);

  char config[2] = {0};
  config[0] = __MODE2;
  config[1] = __OUTDRV;
  write(file, config, 2);
  
  config[0] = __MODE1;
  config[1] = __ALLCALL;
  write(file, config, 2);
  sleep(1);

 
  short mode1;
  
  mode1 = read(file, __MODE1, 1);

  printf("Mode1: %d\n",mode1);
  
  mode1 = mode1 & ~__SLEEP;
  printf("Mode1: %d\n",mode1);
  char reg[1] = {(char)mode1};
  
  config[0] = __MODE1;
  config[1] = mode1;
  write(file, config, 2);
  sleep(1);
  
  char data[1] = {0};
  if(read(file, data, 1) != 1)
    {
      printf("Error : Input/output Error \n");
      //exit(1);
    }
}

void setPWMFreq(int file, short freq){

}

void setAllPWN(short on, short off){
}

void setPWM(short channel, short on, short off){
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
  sleep(1);

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
  acclArray[1] = yAccl;
  acclArray[2] = zAccl;
    
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
  sleep(1);

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
    magArray[0] = xMag;
    magArray[1] = yMag;
    magArray[2] = zMag;
    
    // Output data to screen
    printf("Magnetic field in X-Axis : %d \n", xMag);
    printf("Magnetic field in Y-Axis : %d \n", yMag);
    printf("Magnetic field in Z-Axis : %d \n", zMag);
}
*/
int main() 
{
  // Create I2C bus
  /* int file;
  char *bus = "/dev/i2c-1";
  if((file = open(bus, O_RDWR)) < 0) 
    {
      printf("Failed to open the bus. \n");
      exit(1);
    }
	
  //int pwm;
  //initPWM(pwm, 0x40);

  // Get I2C device PWM
  //ioctl(file, I2C_SLAVE, 0x40);

  int *acclArray,
    *magArray;
  acclArray = malloc(3*sizeof(int));
  magArray = malloc(3*sizeof(int));
  // Loop
  while(1){
    readGyro(file, acclArray);
    readMag(file, magArray);

   
  }*/\
  printf("Hi\n");
  return 0;
}
