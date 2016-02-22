#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import String

def callback(data):
    if data.data == ".":
	GPIO.output(25, False)
	time.sleep(0.2)
	GPIO.output(25, True)
    elif data.data == "_":
	GPIO.output(25, False)
	time.sleep(0.6)
	GPIO.output(25, True)
    else:
    	rospy.loginfo(rospy.get_caller_id())
	GPIO.output(24, True)
	GPIO.output(25, True)
    
def listener():
    # run simultaneously.
    rospy.init_node('erle_statusleds_morse', anonymous=True)
    rospy.Subscriber("statusleds_morse", String, callback)
    # Init corresponding GPIOs
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(24, GPIO.OUT) 
    GPIO.setup(25, GPIO.OUT) 
    # Init them
    GPIO.output(24, True)
    GPIO.output(25, True)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
