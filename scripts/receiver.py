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
zero_twist.linear.x = 0.0
zero_twist.angular.z = 0.0
force_stop = False


def set_speed(data):
    global vel
    vel = data
def wheel_callback(data):
    global force_stop
    if data.state==WheelDropEvent.DROPPED:
        force_stop = True
        sound_pub.publish(bsound)
        rospy.logerr("wheel force stop")
    else: force_stop = False

def bumper_callback(data):
    global force_stop
    if data.state==1:
        force_stop = True
        sound_pub.publish(bsound)
        rospy.logerr("bumper force stop")
    else: force_stop = False


rate = rospy.Rate(30)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=2)
vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=2)
rospy.Subscriber('/mobile_base/events/wheel_drop', WheelDropEvent, wheel_callback)
rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)
rospy.Subscriber('/final',Twist,set_speed)

while not rospy.is_shutdown():
    if force_stop and vel.linear.x > 0:
        vel_pub.publish(zero_twist)
    else: vel_pub.publish(vel)
    rate.sleep()