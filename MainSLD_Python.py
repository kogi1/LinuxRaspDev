#!/usr/bin/env python
# -*- coding: utf8 -*-

import RPi.GPIO as GPIO
import MFRC522
import signal

import requests

from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD

from time import sleep, strftime
from datetime import datetime

PCF8574_address = 0x27  # I2C address of the PCF8574 chip.lul
PCF8574A_address = 0x3F  # I2C  address of the PCF8574A chip.

pins = {'pin_R': 33, 'pin_G': 12, 'pin_B': 13}  # pins is a dict
buzzerPin = 36

statusMessage = False
color_Green = 0x00FF40
color_Orange = 0xFFBF00
color_Red = 0xFF0000
color_Blue = 0x0000FF


def setup():
	global p_R,p_G,p_B
	print('Program is starting ... ')
	GPIO.setmode(GPIO.BOARD)       # Numbersads GPIOs by physical location
	GPIO.setup(buzzerPin, GPIO.OUT)
	for i in pins:
		GPIO.setup(pins[i], GPIO.OUT)   # Set pins' mode is output
		GPIO.output(pins[i], GPIO.HIGH)  # Set pins to high(+3.3V) to off led
	p_R = GPIO.PWM(pins['pin_R'], 2000)  # set Frequece to 2KHz
	p_G = GPIO.PWM(pins['pin_G'], 2000)
	p_B = GPIO.PWM(pins['pin_B'], 2000)
	p_R.start(0)      # Initial duty Cycle = 0
	p_G.start(0)
	p_B.start(0)
# Create PCF8574 GPIO adapter.


try:
	mcp = PCF8574_GPIO(PCF8574_address)
except:
	try:
		mcp = PCF8574_GPIO(PCF8574A_address)
	except:
		print('I2C Address Error !')
		exit(1)
# Create LCD, passing in MCP GPIO adapter.
lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4, 5, 6, 7], GPIO=mcp)

continue_reading = True


# Capture SIGINT for cleanup when the script is aborted
def destroy():
	p_R.stop()
	p_G.stop()
	p_B.stop()
	GPIO.output(buzzerPin, GPIO.LOW)
	GPIO.cleanup()


def end_read(signal, frame):
	global continue_reading
	print("Ctrl+C captured, ending read.")
	continue_reading = False
	GPIO.cleanup()
	destroy()


# Hook the SIGINT
signal.signal(signal.SIGINT, end_read)

mcp.output(3, 1)     # turn on LCD backlight
lcd.begin(16, 2)     # set number of LCD lines and columns

# Create an object of the class MFRC522
MIFAREReader = MFRC522.MFRC522()

# Welcome message
setup()
print("Welcome to the MFRC522 data read example")
print("Press Ctrl-C to stop.")


def map(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def setColor(col):  # For example : col = 0x112233
	R_val = (col & 0x110000) >> 16
	G_val = (col & 0x001100) >> 8
	B_val = (col & 0x000011) >> 0

	R_val = map(R_val, 0, 255, 0, 100)
	G_val = map(G_val, 0, 255, 0, 100)
	B_val = map(B_val, 0, 255, 0, 100)

	p_R.ChangeDutyCycle(100-R_val)     # Change duty cycle
	p_G.ChangeDutyCycle(100-G_val)
	p_B.ChangeDutyCycle(100-B_val)


def ton():
	GPIO.output(buzzerPin,GPIO.HIGH)
	sleep(0.3)
	GPIO.output(buzzerPin,GPIO.LOW)


def STATMess():
	if statusMessage == False:
		global mes
		mes = "   SolidShot   "
		lcd.clear()
		lcd.message(mes+'\n')
		lcd.message(datetime.now().strftime('  %d %b %Y'))
		setColor(color_Blue)


# This loop keeps checking for chips. If one is near it will get the UID and authenticate
while continue_reading:
	STATMess()
	statusMessage = True
	# Scan for cards
	(status, TagType) = MIFAREReader.MFRC522_Request(MIFAREReader.PICC_REQIDL)
# If a card is found
	if status == MIFAREReader.MI_OK:
		print("Card detected")
		continue_reading = False
# Get the UID of the card
		(status, uid) = MIFAREReader.MFRC522_Anticoll()
# If we have the UID, continue

	if status == MIFAREReader.MI_OK:
		lcd.clear()
		print("Card read UID: "+str(uid[0])+","+str(uid[1])+","+str(uid[2])+","+str(uid[3]))
		lcd.setCursor(0,0)
		lcd.message('Reading...')
		setColor(color_Orange)
		ton()
		sleep(0.5)
		lcd.clear()

		uidtosend = str(uid[0])+"-"+str(uid[1])+"-"+str(uid[2])+"-"+str(uid[3])
		params = {'uid': uidtosend}
		response = requests.get('https://admin.solidshot.at/DBA/uid.php', params=params)
		print(response.text)

		setColor(color_Green)

		lcd.message('Card read UID:'+'\n')  # display CPU temperature
		lcd.message(str(uid[0])+","+str(uid[1])+","+str(uid[2])+","+str(uid[3]))
		sleep(3)
		lcd.clear()
		continue_reading = True
		statusMessage = False
