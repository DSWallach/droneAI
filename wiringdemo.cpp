#include <stdio.h>    // Used for printf() statements
#include <wiringPi.h> // Include WiringPi library!

// Pin number declarations. We're using the Broadcom chip pin numbers.
const int r = 17,
  g = 20,
  b = 21; // PWM LED - Broadcom pin 18, P1 pin 12
const int ledPin = 23; // Regular LED - Broadcom pin 23, P1 pin 16
const int butPin = 17; // Active-low button - Broadcom pin 17, P1 pin 11

const int pwmValue = 1024; // Use this to set an LED brightness

int main(void)
{
  // Setup stuff:
  wiringPiSetupGpio(); // Initialize wiringPi -- using Broadcom pin numbers

  pinMode(r, PWM_OUTPUT); // Set PWM LED as PWM output
  pinMode(g, PWM_OUTPUT); // Set PWM LED as PWM output
  pinMode(b, PWM_OUTPUT); // Set PWM LED as PWM output

  printf("Blinker is running! Press CTRL+C to quit.\n");

  // Loop (while(1)):
  while(1)
    {
      for (int xi = 0; xi < pwmValue; xi++){
	/*	for (int yi = 0; yi < pwmValue; yi++){
	  for (int zi = 0; zi <pwmValue; zi++){
            pwmWrite(r, xi); // PWM LED at bright setting
	    pwmWrite(g, yi);
	    pwmWrite(b, zi);
	    printf("r: %d, g: %d, b: %d\n",xi,yi, zi);
	    delay(5);
	    pwmWrite(r, 0);
	    pwmWrite(g, 0);
	    pwmWrite(b, 0);
	  }
	}*/
	for (int yi = 0; yi < 2; yi++){
	  for (int zi = 0; zi < 2; zi++){
            digitalWrite(r, HIGH); // PWM LED at bright setting
	    digitalWrite(g, HIGH);
	    digitalWrite(b, HIGH);
	    delay(5);
	    printf("digi r: %d, g: %d, b: %d\n",xi,yi, zi);
	    digitalWrite(r, LOW);
	    digitalWrite(g, LOW);
	    digitalWrite(b, LOW);
	  }
	}
      }
    }

  return 0;
}
