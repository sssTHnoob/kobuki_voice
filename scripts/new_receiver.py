#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Sound
from kobuki_msgs.msg import WheelDropEvent
from kobuki_msgs.msg import BumperEvent
rospy.init_node('final')
sound = Sound(value=Sound.CLEANINGSTART)
bsound = Sound(value=Sound.BUTTON)
vel = Twist()
zero_twist = Twist()
bump_stop = False
wheel_stop = False
wheel = [0,0]
bumper = [0,0,0]

def set_speed(data):
    global vel
    vel = data
def wheel_callback(data):
    global wheel_stop, wheel
    wheel[data.wheel] = data.state
    if wheel != [0,0]: wheel_stop = True
    else: wheel_stop = False
    if data.state==WheelDropEvent.DROPPED:
        sound_pub.publish(bsound)
        rospy.logerr("wheel force stop")

def bumper_callback(data):
    global bump_stop, bumper
    bumper[data.bumper] = data.state
    if bumper != [0,0,0]: bump_stop = True
    else: bump_stop = False
    if data.state==1:
        sound_pub.publish(bsound)
        rospy.logerr("bumper force stop")


rate = rospy.Rate(30)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
rospy.Subscriber('/mobile_base/events/wheel_drop', WheelDropEvent, wheel_callback)
rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)
rospy.Subscriber('/final',Twist,set_speed)
while not rospy.is_shutdown():
    if bump_stop or wheel_stop: vel_pub.publish(zero_twist)
    else: vel_pub.publish(vel)
    rate.sleep()