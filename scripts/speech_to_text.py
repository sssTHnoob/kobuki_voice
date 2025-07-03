#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import os
import datetime
import errno
import re

def speech_recognizer_node():
    #initial setup
    if True:
        rospy.init_node('speech_recognizer', anonymous=True)
        pub = rospy.Publisher('recognized_speech', String, queue_size=10)

        r = sr.Recognizer()
        r.dynamic_energy_threshold = False
        # r.energy_threshold = 300 # Adjust if too sensitive or not sensitive enough to speech
        # r.dynamic_energy_threshold = True # Recommended for varying noise levels
        # r.pause_threshold = 0.8 # Seconds of non-speaking audio before a phrase is considered complete
        script_dir = os.path.dirname(os.path.abspath(__file__))
        package_root = os.path.abspath(os.path.join(script_dir, os.pardir))
        recordings_dir = os.path.join(package_root, "recordings")
        try:
            os.makedirs(recordings_dir)
        except OSError as e:
            if e.errno != errno.EEXIST:  # If error is not "directory already exists"
                raise
        rospy.logwarn("Audio recordings will be saved to: {}".format(recordings_dir))

        try:
            with sr.Microphone() as source:
                rospy.logwarn("Calibrating microphone for ambient noise... Please be quiet for a moment.")
                r.adjust_for_ambient_noise(source, duration=3)  # You can increase duration (e.g., 2-3 seconds)
                rospy.logwarn("Calibration complete! Set energy threshold to: {}".format(r.energy_threshold))
                rospy.logwarn("You can now start speaking.")
        except Exception as e:
            rospy.logerr("Error during initial microphone calibration: {0}. Exiting.".format(e))
            return  # Exit the node if calibration fails
    while not rospy.is_shutdown():
        try:
            with sr.Microphone() as source:
                rospy.loginfo("Listening...")
                audio = r.listen(source)
            rospy.loginfo("Processing audio...")
            #text = r.recognize_google(audio) or _sphinx
            # Option 2: CMU PocketSphinx (offline)
            # You might need to specify a language model if not using default English
            # For custom models, you'd pass model_dir, language, etc.
            text = r.recognize_google(audio,language="en-US")
            pub.publish(text)
            ###########################################################################################################
            if audio:  # Ensure directory is set and audio data exists
                if text:  # If text was successfully recognized (not empty)
                    # Sanitize the recognized text for use in a filename
                    sanitized_text = re.sub(r'[^\w\s-]', '', text).strip()
                    sanitized_text = re.sub(r'[-\s]+', '_', sanitized_text)
                    sanitized_text = sanitized_text[:50]

                    if not sanitized_text:  # Fallback if sanitized text becomes empty (e.g., "!")
                        sanitized_text = "empty_sanitized_text"  # Name for cases like punctuation-only input
                    timestamp_suffix = datetime.datetime.now().strftime("_%H%M%S")
                    filename = os.path.join(recordings_dir, "{}{}.wav".format(sanitized_text, timestamp_suffix))
                    try:
                        with open(filename, "wb") as f:
                            f.write(audio.get_wav_data())
                    except Exception as e:
                        rospy.logerr("Error saving recognized audio file: {0}".format(e))

                else:  # text is empty, but no UnknownValueError was raised
                    rospy.logwarn("Recognized text was empty.")
                    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = os.path.join(recordings_dir, "empty_text_{}.wav".format(timestamp))
                    try:
                        with open(filename, "wb") as f:
                            f.write(audio.get_wav_data())
                    except Exception as e:
                        rospy.logerr("Error saving empty text audio file: {0}".format(e))
#######################################################################################################################
        except sr.UnknownValueError:
            rospy.logwarn("Could not understand audio")
            if recordings_dir and audio:  # Ensure directory is set and audio data exists
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(recordings_dir, "unrecognized_{}.wav".format(timestamp))
                try:
                    with open(filename, "wb") as f:
                        f.write(audio.get_wav_data())
                except Exception as e:
                    rospy.logerr("Error saving unrecognized audio file: {0}".format(e))
            elif not recordings_dir:
                rospy.logwarn("Recordings directory not set, skipping unrecognized audio save.")
        except sr.RequestError as e:
            rospy.logerr("Could not request results from service; {0}".format(e))
        except Exception as e:
            rospy.logerr("An error occurred: {0}".format(e))

        rospy.sleep(0.01) # Small delay to prevent high CPU usage in continuous loop

if __name__ == '__main__':
    try:
        speech_recognizer_node()
    except rospy.ROSInterruptException:
        pass