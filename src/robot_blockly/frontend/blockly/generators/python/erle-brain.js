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
 * @author victor@erlerobot.com (Víctor Mayoral Vilches)
 */
'use strict';

goog.provide('Blockly.Python.brain');
goog.require('Blockly.Python');

Blockly.Python['turn_on_blue_led'] = function(block) {
        var blue_led = block.getFieldValue('BLUE_LED');
	var code = "";
	code+="#!/usr/bin/python\n"
	code+="import rospy\n"
	code+="import time\n"
	code+="from std_msgs.msg import String\n"
	code+="\n"
	code+="def talker():\n"
	code+="  pub = rospy.Publisher('/statusleds', String, queue_size=10)\n"
	code+="  rate = rospy.Rate(10)\n"
	code+="  start = time.time()\n"
	code+="  flag=True #time flag\n"
        code+="  led='"+blue_led.toString()+"'\n"
	code+="  if (led == 'TRUE'):\n"
        code+="    msg = 'blue'\n"
        code+="  else:\n"
        code+="    msg = 'blue_off'\n"
	code+="  while not rospy.is_shutdown() and flag:\n"
	code+="    sample_time=time.time()\n"
  	code+="    if ((sample_time - start) > 1):\n"
	code+="      flag=False\n"
	code+="    pub.publish(msg)\n"
	code+="    rate.sleep()\n"
	code+="if __name__ == '__main__':\n"
	code+="  talker()\n"	
        return code;

};

Blockly.Python['turn_on_orange_led'] = function(block) {
        var orange_led = block.getFieldValue('ORANGE_LED');
        var code = "";
        code+="#!/usr/bin/python\n"
        code+="import rospy\n"
        code+="import time\n"
        code+="from std_msgs.msg import String\n"
        code+="\n"
        code+="def talker():\n"
        code+="  pub = rospy.Publisher('/statusleds', String, queue_size=10)\n"
        code+="  rate = rospy.Rate(10)\n"
        code+="  start = time.time()\n"
        code+="  flag=True #time flag\n"
        code+="  led='"+orange_led.toString()+"'\n"
        code+="  if (led == 'TRUE'):\n"
        code+="    msg = 'orange'\n"
        code+="  else:\n"
        code+="    msg = 'orange_off'\n"
        code+="  while not rospy.is_shutdown() and flag:\n"
        code+="    sample_time=time.time()\n"
        code+="    if ((sample_time - start) > 1):\n"
        code+="      flag=False\n"
        code+="    pub.publish(msg)\n"
        code+="    rate.sleep()\n"
        code+="if __name__ == '__main__':\n"
        code+="  talker()\n"
        return code;

};

Blockly.Python['text_to_morse'] = function(block) {
        var text = block.getFieldValue('TEXT');
	
	var alphabet = {
    	'a': '.-',    'b': '-...',  'c': '-.-.', 'd': '-..',
    	'e': '.',     'f': '..-.',  'g': '--.',  'h': '....',
    	'i': '..',    'j': '.---',  'k': '-.-',  'l': '.-..',
    	'm': '--',    'n': '-.',    'o': '---',  'p': '.--.',
    	'q': '--.-',  'r': '.-.',   's': '...',  't': '-',
    	'u': '..-',   'v': '...-',  'w': '.--',  'x': '-..-',
    	'y': '-.--',  'z': '--..',  ' ': '/',
    	'1': '.----', '2': '..---', '3': '...--', '4': '....-', 
    	'5': '.....', '6': '-....', '7': '--...', '8': '---..', 
	'9': '----.', '0': '-----', 
	}

	var v0 = text.split('');            // Transform the string into an array: ['T', 'h', 'i', 's'...
    	var v1 = v0.map(function(e){     // Replace each character with a morse "letter"
        	return alphabet[e.toLowerCase()] || ''; // Lowercase only, ignore unknown characters.
    	});
    	var v2 = v1.join(' ');            // Convert the array back to a string.
    	var morse_string = v2.replace(/ +/g, ' '); // Replace double spaces that may occur when unknow characters were in the source string.
	
	alert("MORSE CODE: "+morse_string);

        var code = "";
        code+="#!/usr/bin/python\n"
        code+="import rospy\n"
        code+="import time\n"
        code+="from std_msgs.msg import String\n"
        code+="\n"
        code+="def talker():\n"
        code+="  pub = rospy.Publisher('/statusleds_morse', String, queue_size=10)\n"
        code+="  rate = rospy.Rate(10)\n"
        code+="  start = time.time()\n"
        code+="  flag=True #time flag\n"
        code+="  morseCode='."+morse_string.toString()+"' #add . because first char gets lost\n"
        code+="  while not rospy.is_shutdown() and flag:\n"
        code+="    sample_time=time.time()\n"
        code+="    if ((sample_time - start) > 1):\n"
        code+="      flag=False\n"
        code+="    flag=False\n"
	code+="    for c in morseCode:\n"
	code+="      msg = c\n"
	code+="      pub.publish(msg)\n"
        code+="      rate.sleep()\n"
        code+="    rate.sleep()\n"
	code+="\n"
        code+="if __name__ == '__main__':\n"
        code+="  talker()\n"
        return code;

};

