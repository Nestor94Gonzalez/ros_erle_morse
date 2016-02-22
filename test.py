#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node('blockly_node', anonymous=True)
#!/usr/bin/python
import rospy
import time
from std_msgs.msg import String

def talker():
  pub = rospy.Publisher('/statusleds', String, queue_size=10)
  rate = rospy.Rate(10)
  start = time.time()
  flag=True #time flag
  led='TRUE'
  if (led == 'TRUE'):
    msg = 'orange'
  else:
    msg = 'orange_off'
  while not rospy.is_shutdown() and flag:
    sample_time=time.time()
    if ((sample_time - start) > 1):
      flag=False
    pub.publish(msg)
    rate.sleep()
if __name__ == '__main__':
  talker()
#!/usr/bin/python
import rospy
import time
from std_msgs.msg import String

def talker():
  pub = rospy.Publisher('/statusleds_morse', String, queue_size=10)
  rate = rospy.Rate(10)
  start = time.time()
  flag=True #time flag
  morseCode='.---' #add . because first char gets lost
  while not rospy.is_shutdown() and flag:
    sample_time=time.time()
    if ((sample_time - start) > 1):
      flag=False
    flag=False
    for c in morseCode:
      msg = c
      pub.publish(msg)
      rate.sleep()
    rate.sleep()

if __name__ == '__main__':
  talker()


