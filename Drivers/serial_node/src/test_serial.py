#!/usr/bin/python

import time
import serial
import sys

#whis = open('./whisker.txt','a') #opens data file, arg 'a' opens the file for appending data

#################################################################	
def write_whis(val):
	#write best fitness of current population to file
	strval = str(val) #convert the value to a string, may not be needed
	whis.write(strval) #write string to file
	whis.write('\n') #new line the file	
#################################################################


# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
	port ='/dev/ttyS0',
	baudrate = 38400,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS
)

ser.open()
ser.isOpen()

print"Serial Comms opened"

#ser.read() #how to read in from the serial port

ser.write("\x00\x05\x04\xbf\x71")

for i in range(0,300):
	x[i] = ser.read() #read the serial and put current value into x[i]
	#write_whis(x[i]) #write the value to a text file to 
	print x[i] #print it on the screen

print "finished sweep"

