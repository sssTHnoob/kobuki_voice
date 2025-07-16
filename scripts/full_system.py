#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import speech_recognition as sr
from speech_recognition import WaitTimeoutError
import multiprocessing
from geometry_msgs.msg import Twist

rospy.init_node('main')
vel_pub = rospy.Publisher('final', Twist, queue_size=10)
r = sr.Recognizer()
r.pause_threshold = 0.5
r.dynamic_energy_threshold = False
command = False
stop_twist = Twist()


def speech_recog(lang):
    global command
    try:
        text = r.recognize_google(audio, language=lang).encode('utf-8').lower()
        rospy.loginfo("%s: %s", lang, text)
        if "start command" in text:
            command = True
            vel_pub.publish(stop_twist)

    except sr.UnknownValueError: pass

with sr.Microphone(device_index=2) as source:
    rospy.logwarn("Calibrating microphone for ambient noise... Please be quiet for a moment.")
    r.adjust_for_ambient_noise(source, duration=3) #r.energy_threshold += 100
    rospy.logwarn("Calibration complete! Set energy threshold to: {}".format(r.energy_threshold))
    rospy.logwarn("You can now start speaking.")

while not rospy.is_shutdown():
    try:
        with sr.Microphone(device_index=2) as source:
            audio = r.listen(source)
        multiprocessing.Process(target=speech_recog, args=("en-US",)).start()
        multiprocessing.Process(target=speech_recog, args=("th-TH",)).start()
    except WaitTimeoutError: pass