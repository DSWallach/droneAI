#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  RGB_LED.py
#
# A short program to control an RGB LED by utilizing
# the PWM functions within the Python GPIO module
#
#  Copyright 2015  Ken Powers
#   
 
# Import the modules used in the script
import random, time
import RPi.GPIO as GPIO

GPIO.cleanup()

# Set GPIO to Broadcom system and set RGB Pin numbers
RUNNING = True
GPIO.setmode(GPIO.BCM)

FR = 4
BR = 17
FL = 20
BL = 21

# Set pins to output mode
GPIO.setup(FR, GPIO.OUT)
GPIO.setup(BR, GPIO.OUT)
GPIO.setup(FL, GPIO.OUT)
GPIO.setup(BL, GPIO.OUT)

Freq = 120 #Hz
 
# Setup all the LED colors with an initial
# duty cycle of 0 which is off
FRM = GPIO.PWM(FR, Freq)
FRM.start(0)
BRM = GPIO.PWM(BR, Freq)
BRM.start(0)
FLM = GPIO.PWM(FL, Freq)
FLM.start(0)
BLM = GPIO.PWM(BL, Freq)
BLM.start(0)

# Define a simple function to turn on the LED colors
def startMotor(FRS, BRS, FLS, BLS, on_time):
    # Color brightness range is 0-100%
    FRM.ChangeDutyCycle(FRS)
    BRM.ChangeDutyCycle(BRS)
    FLM.ChangeDutyCycle(FLS)
    BLM.ChangeDutyCycle(BLS)
    time.sleep(on_time)

def stopMotor():
    # Turn all LEDs off after on_time seconds
    FRM.ChangeDutyCycle(0)
    BRM.ChangeDutyCycle(0)
    FLM.ChangeDutyCycle(0)
    BLM.ChangeDutyCycle(0)
 
print("Light It Up!")
print("Press CTRL + C to quit.\n")
print("FR BR FL BL\n---------")
x = y = z = w = 1
# Main loop
try:
    while RUNNING:
        # Slowly ramp up power percentage of each active color
        for i in range(20):
            #startMotor((w*(5+i)),(x*(0+i)),(y*(5+i)),(z*(5+i)), .1)
            startMotor((1*(5+i)),(1*(0+i)),(1*(5+i)),(1*(5+i)), .05)
        time.sleep(1)

                        
# If CTRL+C is pressed the main loop is broken
except KeyboardInterrupt:
    RUNNING = False
    print "\Quitting"
 
# Actions under 'finally' will always be called
# regardless of what stopped the program
finally:
    # Stop and cleanup so the pins
    # are available to be used again
    stopMotor()
    GPIO.cleanup()
