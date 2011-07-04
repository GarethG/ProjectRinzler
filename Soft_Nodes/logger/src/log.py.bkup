#!/usr/bin/env python

#The format of the log file will be a comma separated ASCII file of the format:
#Time,     position,      action, a comment between simple quotes.
#(SSSSS,XXX.x,YYY.y,ZZZ.z,AA.aa). Logged data will be plotted by plotting routine
#written by the organising committee


import sys
import time
import roslib; roslib.load_manifest('logger') #is this the right manifest file?
import rospy


#from std_msgs.msg import String

logtime = time.strftime("%d.%m.%y.%H.%M") #get the current system time in the format DD.MM.YY , HH.MM
#the file() creator can only take one string argument so construct a filename
logname = 'UweSub_logfile ' + logtime + '.txt'

log = open(logname, 'a')

#------------------------------------------------
#A Function to write a single entry to the file
def writeEntry(entry):
	logEntry = str(entry) #entry should already be in a string at this point, but just to be sure
	log.write(logEntry)
	log.write('\n')
#-----------------------------------------------
#for the time being make them these 
navXPose = 112
navYPose = 300
navZPose = 3000
action = 'diving'
#this will all be inside a rosspin, the one to control how many times it runs 

sss = time.strftime("%d.%m.%y.%H.%M")
xxx = str(navXPose) + '.x'
yyy = str(navYPose) + '.y'
zzz = str(navZPose) + '.z'
aa  = str(action)   + '.aa'

entry = sss + ',' + xxx + ',' + yyy + ',' + zzz + ',' + aa   
writeEntry(entry)


