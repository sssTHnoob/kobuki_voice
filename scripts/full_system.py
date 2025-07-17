#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import speech_recognition as sr
import threading
from std_msgs.msg import String
from speech_recognition import WaitTimeoutError
rospy.init_node('speech_receive')

pub = rospy.Publisher('recog_speech', String, queue_size=5)
comm = rospy.Publisher('set_command', String,queue_size=5)
r = sr.Recognizer()
r.non_speaking_duration = 0.4
r.pause_threshold = 0.4
r.dynamic_energy_threshold = False
chained = False

def callback(data):
    global chained
    audio = data
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

with sr.Microphone(device_index=2) as source:
    rospy.logwarn("Calibrating microphone for ambient noise... Please be quiet for a moment.")
    r.adjust_for_ambient_noise(source, duration=3)
    #r.energy_threshold += 100
    rospy.logwarn("Calibration complete! Set energy threshold to: {}".format(r.energy_threshold))
    rospy.logwarn("You can now start speaking.")

while not rospy.is_shutdown():
    try:
        with sr.Microphone(device_index=2) as source:
            audio_receive = r.listen(source)
        threading.Thread(target=callback, args=(audio_receive,)).start()
    except WaitTimeoutError: pass