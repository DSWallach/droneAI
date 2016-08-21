CC = g++ -Og -lwiringPi
NAVIO = Navio

all:
	$(CC) Servo.cpp $(NAVIO)/PCA9685.cpp $(NAVIO)/I2Cdev.cpp $(NAVIO)/gpio.cpp $(NAVIO)/Util.cpp -o Servo.o
	$(CC) wiringdemo.cpp -o demo.o
	gcc -Og -lm -lwiringPi drone.c -o drone.o
drone:
	gcc -Og -lm -lwiringPi drone.c -o drone.o
demo:
	$(CC) wiringdemo.cpp -o demo.o

clean:
	rm *.o
