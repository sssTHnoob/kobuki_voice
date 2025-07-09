#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import speech_recognition as sr
from audio_common_msgs.msg import AudioData
rospy.init_node('speech_recognizer')

r=sr.Recognizer()
chained = False

pub = rospy.Publisher('recog_test', String, queue_size=10)
comm = rospy.Publisher('set_command', String,queue_size=10)
def callback(data):
    global chained
    audio = sr.AudioData(data.data, 44100, 2)
    text = ""
    if not chained:
        try:
            text = r.recognize_google(audio, language="en-US").lower()
            if "command" in text:
                pub.publish("command start")
                chained = True
            else: pub.publish(text)
        except sr.UnknownValueError: rospy.loginfo("try speak english again")
        if not chained:
            try:
                tt = r.recognize_google(audio, language="th-TH").lower()
                if u"ชุด" in tt or "command" in tt:
                    pub.publish("command start")
                    chained = True
                elif tt.lower() not in text.lower(): pub.publish(tt)
            except sr.UnknownValueError: rospy.loginfo("try speak thai again")
    else:
        try: text += r.recognize_google(audio, language="en-US").lower()
        except sr.UnknownValueError: pass
        try: text += r.recognize_google(audio, language="th-TH").lower()
        except sr.UnknownValueError: pass
        comm.publish(text)
        if "command" in text:
            chained = False
rospy.Subscriber('/speech',AudioData,callback)
rospy.spin()
