#!/usr/bin/python
import rospy
import time
from std_msgs.msg import String

def talker():
  pub = rospy.Publisher('/statusleds_morse', String, queue_size=10)
  rospy.init_node('erle_statusleds_morse')
  rate = rospy.Rate(10)
  start = time.time()
  flag=True #time flag
  morseCode='-'
  while not rospy.is_shutdown() and flag:
    sample_time=time.time()
    if ((sample_time - start) > 0.5):
      flag=False
    for c in morseCode:
      msg = c
      pub.publish(msg)
      rate.sleep()
    rate.sleep()

if __name__ == '__main__':
  talker()

