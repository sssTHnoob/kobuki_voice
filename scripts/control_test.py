#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Sound
import pyttsx3
import threading
rospy.init_node("controller")

vel = Twist()
out = Twist()
sound = Sound(value=Sound.CLEANINGSTART)
bsound = Sound(value=Sound.BUTTON)
speaker = pyttsx3.init()
command_run = False


f = 1
def speak_func(text):
    speaker.say(text)
    speaker.runAndWait()
    speaker.stop()
def speech_callback(data):
    global vel, sound, f, command_run
    recognized_text = data.data.lower()

    rospy.loginfo("Received command: %s", recognized_text)
    if "command" in recognized_text:
        vel.linear.x = 0
        vel.angular.z = 0
        vel_pub.publish(vel)
        rospy.loginfo("stop robot")
        command_run = not command_run
    elif "move forward" in recognized_text or "เดินหน้า" in recognized_text:
        vel.linear.x = 0.1
        vel.angular.z = 0.0
        threading.Thread(target=speak_func, args=("move forward",)).start()
    elif "move backward" in recognized_text or "ถอยหลัง" in recognized_text:
        vel.linear.x = -0.1
        vel.angular.z = 0.0
        threading.Thread(target=speak_func, args=("move backward",)).start()
    elif "turn left" in recognized_text or "ซ้าย" in recognized_text:
        vel.linear.x = 0.0
        vel.angular.z = 0.5
        threading.Thread(target=speak_func, args=("turn left",)).start()
    elif "turn right" in recognized_text or "ขวา" in recognized_text:
        vel.linear.x = 0.0
        vel.angular.z = -0.5
        threading.Thread(target=speak_func, args=("turn right",)).start()
    elif "stop" in recognized_text or "หยุด" in recognized_text:
        vel.linear.x = 0
        vel.angular.z = 0
        threading.Thread(target=speak_func, args=("stop",)).start()
    elif "speed up" in recognized_text or "เร่ง" in recognized_text or "เพิ่ม" in recognized_text:
        f+=1
        threading.Thread(target=speak_func, args=("speed level" + str(f),)).start()
    elif "speed down" in recognized_text or "ลด" in recognized_text:
        if f>1:
            f -= 1
            threading.Thread(target=speak_func, args=("speed level" + str(f),)).start()
    elif "reset" in recognized_text or "รีเซ็ต" in recognized_text:
        f = 1
        threading.Thread(target=speak_func, args=("speed level reset",)).start()



rate = rospy.Rate(20)
vel_pub = rospy.Publisher('/final', Twist, queue_size=10)
rospy.Subscriber('/recognized_speech', String, speech_callback)
rospy.Subscriber('/recognized_speech_command', String, speech_callback)
while not rospy.is_shutdown():
    if not command_run:
        out.linear.x = vel.linear.x * f
        out.angular.z = vel.angular.z * f
        vel_pub.publish(out)
        rate.sleep()