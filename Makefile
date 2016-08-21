CC = g++ -Og
NAVIO = Navio

all:
	$(CC) Servo.cpp $(NAVIO)/PCA9685.cpp $(NAVIO)/I2Cdev.cpp $(NAVIO)/gpio.cpp $(NAVIO)/Util.cpp -o Servo
	$(CC) drone.cpp -o drone

clean:
	rm Servo
	rm drone
