/**
 * @license
 *
 * Copyright 2015 Erle Robotics
 * http://erlerobotics.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview Blocks for Erle-Spider.
 * @author inigo@erlerobot.com (Iñigo Muguruza Goenaga)
 */
'use strict';

goog.provide('Blockly.Python.rover');
goog.require('Blockly.Python');



Blockly.Python['rover_mode'] = function(block) {
  var dropdown_mode = block.getFieldValue('MODE')
  
    var code = ""
    code+="import rospy\n"
    code+="from mavros_msgs.srv import SetMode\n"
    //code+="rospy.init_node('rover_mode')\n"
    code+="rospy.wait_for_service('/mavros/set_mode')\n"
    code+="try:\n"
    code+=" change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)\n"
    code+=" mode='"+dropdown_mode.toString()+"'\n"
    code+=" resp1 = change_mode(0,mode)\n"
    code+="except rospy.ServiceException as e:\n"
    code+=' print ("Service call failed: %s" %e\n )'
    code+="\n"

  return code;
};

Blockly.Python['rover_forward'] = function(block) {
  var dropdown_speed = block.getFieldValue('SPEED');
  var value_time = Blockly.Python.valueToCode(block, 'TIME', Blockly.Python.ORDER_ATOMIC);
  var code = ""
  code+="import rospy\n"
  code+="import time\n"
  code+="from mavros_msgs.msg import OverrideRCIn\n"
  code+="from mavros_msgs.srv import SetMode\n"
  code+="throttle_channel=2\n"
  code+="steer_channel=0"
  code+="def talker():\n"
  code+=" pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)\n"
  //code+=" rospy.init_node('custom_talker', anonymous=True)\n"
  code+=" r = rospy.Rate(10) #10hz\n"
  code+=" msg = OverrideRCIn()\n"
  code+=" start = time.time()\n"
  code+=" speed='"+dropdown_speed.toString()+"'\n"
  code+=" exec_time="+value_time.toString()+"\n"
  code+=" flag=True #time flag\n"
  code+=" msg.channels[steer_channel]=1370\n"
  code+=" if speed =='SLOW':\n"
  code+="  msg.channels[throttle_channel]=1560\n"
  code+=" elif speed =='NORMAL':\n"
  code+="  msg.channels[throttle_channel]=1565\n"
  code+=" elif speed == 'FAST':\n"
  code+="  msg.channels[throttle_channel]=1570\n"
  code+=" while not rospy.is_shutdown() and flag:\n"
  code+="  sample_time=time.time()\n"
  code+="  if ((sample_time - start) > exec_time):\n"
  code+="   flag=False\n"
  code+="  rospy.loginfo(msg)\n"
  code+="  pub.publish(msg)\n"
  code+="  r.sleep()\n"
  code+="if __name__ == '__main__':\n"
  code+=" rospy.wait_for_service('/mavros/set_mode')\n"
  code+=" change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)\n"
  code+=" resp1 = change_mode(custom_mode='manual')\n"
  code+=" print (resp1)\n"
  code+=" if 'True' in str(resp1):\n"
  code+="  try:\n"
  code+="   talker()\n"
  code+="  except rospy.ROSInterruptException: pass\n"
  code+="\n"


  return code;
};


Blockly.Python['rover_steering'] = function(block) {
  var dropdown_steering = block.getFieldValue('direction');
  var code = ""
  code+="import rospy\n"
  code+="import time\n"
  code+="from mavros_msgs.msg import OverrideRCIn\n"
  code+="from mavros_msgs.srv import SetMode\n"
  code+="throttle_channel=2\n"
  code+="steer_channel=0\n"
  code+="exec_time=1 #exec time in secs\n"
  //code+="'''PWM values are hardcoded right now.\n"
  //code+="Best practices could be to read the max and min\n"
  //code+="of steer and throttle in order to calculate PWM values'''\n"
  code+="def talker_steering():\n"
  code+=" pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)\n"
  //code+=" rospy.init_node('custom_talker', anonymous=True)\n"
  code+=" r = rospy.Rate(10) #10hz\n"
  code+=" msg = OverrideRCIn()\n"
  code+=" start = time.time()\n"
  code+=" flag=True #time flag\n"
  code+=" direction='"+dropdown_steering.toString()+"'\n"
  //code+="\n"
  code+=" if direction =='STRAIGHT':\n"
  code+="  msg.channels[steer_channel]=1385\n"
  code+=" elif direction =='RIGHT':\n"
  code+="  msg.channels[steer_channel]=1450\n"
  code+=" elif direction == 'LEFT':\n"
  code+="  msg.channels[steer_channel]=1250\n"
  code+=" while not rospy.is_shutdown() and flag:\n"
  code+="  sample_time=time.time()\n"
  code+="  if ((sample_time - start) > exec_time):\n"
  code+="   flag=False\n"
  code+="  rospy.loginfo(msg)\n"  
  code+="  pub.publish(msg)\n"
  code+="  r.sleep()\n"
  //code+="\n"
  code+="if __name__ == '__main__':\n"
  code+=" rospy.wait_for_service('/mavros/set_mode')\n"
  code+=" change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)\n"
  code+=" resp2 = change_mode(custom_mode='manual')\n"
  code+=" print (resp2)\n"
  code+=" if 'True' in str(resp2):\n"
  code+="  try:\n"
  code+="   talker_steering()\n"
  code+="  except rospy.ROSInterruptException: pass\n"
  code+="\n"

  return code;
};


Blockly.Python['rover_control'] = function(block) {
  var dropdown_direction = block.getFieldValue('direction');
  var dropdown_speed = block.getFieldValue('speed');
  var value_seconds = Blockly.Python.valueToCode(block, 'seconds', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = ""
  code+="import rospy\n"
  code+="import time\n"
  code+="from mavros_msgs.msg import OverrideRCIn\n"
  code+="from mavros_msgs.srv import SetMode\n"
  code+="throttle_channel=2\n"
  code+="steer_channel=0\n"
   code+="\n"
  code+="def talker():\n"
  code+=" pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)\n"
  code+=" r = rospy.Rate(10) #10hz\n"
  code+=" msg = OverrideRCIn()\n"
  code+=" start = time.time()\n"
  code+=" speed='"+dropdown_speed.toString()+"'\n"
  code+=" exec_time="+value_seconds.toString()+"\n"
  //code+=" msg.channels[steer_channel]=1370\n"
  code+=" flag=True #time flag\n"
  code+=" if speed =='SLOW':\n"
  code+="  msg.channels[throttle_channel]=1558\n"
  code+=" elif speed =='NORMAL':\n"
  code+="  msg.channels[throttle_channel]=1565\n"
  code+=" elif speed == 'FAST':\n"
  code+="  msg.channels[throttle_channel]=1570\n"
  code+=" direction='"+dropdown_direction.toString()+"'\n"
  code+=" if direction =='STRAIGHT':\n"
  code+="  msg.channels[steer_channel]=1385\n"
  code+=" elif direction =='RIGHT':\n"
  code+="  msg.channels[steer_channel]=1450\n"
  code+=" elif direction == 'LEFT':\n"
  code+="  msg.channels[steer_channel]=1300\n"
  code+=" while not rospy.is_shutdown() and flag:\n"
  code+="  sample_time=time.time()\n"
  code+="  if ((sample_time - start) > exec_time):\n"
  code+="   flag=False\n"
  code+="  rospy.loginfo(msg)\n"
  code+="  pub.publish(msg)\n"
  code+="  r.sleep()\n"
  code+="\n"
  code+="if __name__ == '__main__':\n"
  code+=" rospy.wait_for_service('/mavros/set_mode')\n"
  code+=" change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)\n"
  code+=" resp1 = change_mode(custom_mode='manual')\n"
  code+=" print (resp1)\n"
  code+=" if 'True' in str(resp1):\n"
  code+="  try:\n"
  code+="   talker()\n"
  code+="  except rospy.ROSInterruptException: pass\n"
  code+="\n"
  return code;
};
