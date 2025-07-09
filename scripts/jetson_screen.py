#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from kobuki_msgs.msg import WheelDropEvent
from kobuki_msgs.msg import BumperEvent

def speech_callback(data):
    text = data.data
    rospy.loginfo("Received command: %s", text)
def wheel_callback(data):
    if data.state==WheelDropEvent.DROPPED: rospy.logerr("wheel force stop")

def bumper_callback(data):
    if data.state==1: rospy.logerr("bumper force stop")

rospy.Subscriber('/recognized_speech', String, speech_callback)
rospy.Subscriber('/recognized_speech_command', String, speech_callback)
rospy.Subscriber('/mobile_base/events/wheel_drop', WheelDropEvent, wheel_callback)
rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)
rospy.spin()