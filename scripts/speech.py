#!/usr/bin/env python
import rospy
import speech_recognition as sr
from audio_common_msgs.msg import AudioData
from speech_recognition import WaitTimeoutError
rospy.init_node('speech_receive')

pub = rospy.Publisher('speech', AudioData, queue_size=10)
r = sr.Recognizer()
r.pause_threshold = 0.5
r.dynamic_energy_threshold = False

with sr.Microphone(device_index=2) as source:
    rospy.logwarn("Calibrating microphone for ambient noise... Please be quiet for a moment.")
    r.adjust_for_ambient_noise(source, duration=3)
    #r.energy_threshold += 100
    rospy.logwarn("Calibration complete! Set energy threshold to: {}".format(r.energy_threshold))
    rospy.logwarn("You can now start speaking.")

while not rospy.is_shutdown():
    try:
        with sr.Microphone(device_index=2) as source:
            audio = r.listen(source)
        raw_audio_frames_bytes = audio.get_raw_data()
        audio_ros_msg = AudioData()
        audio_ros_msg.data = [ord(b) for b in raw_audio_frames_bytes]
        pub.publish(audio_ros_msg)
    except WaitTimeoutError: pass