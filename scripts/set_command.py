#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time
import re
rospy.init_node('set_command')

command = []
current_x = 0.0
current_y = 0.0
current_yaw = 0.0


def odom_callback(msg):
    global current_x, current_y, current_yaw
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    # Convert quaternion to Euler (roll, pitch, yaw)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw = yaw

def move_distance(distance_m, linear_speed=0.1, tolerance=0.05):
    global current_x, current_y
    initial_x = current_x
    initial_y = current_y

    twist_msg = Twist()
    twist_msg.linear.x = linear_speed if distance_m > 0 else -linear_speed
    twist_msg.angular.z = 0.0
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        distance_moved = math.sqrt((current_x - initial_x)**2 + (current_y - initial_y)**2)

        if abs(distance_m) - distance_moved <= tolerance:
            rospy.loginfo("Reached target distance!")
            break

        vel_pub.publish(twist_msg)
        rate.sleep()

    twist_msg.linear.x = 0.0
    vel_pub.publish(twist_msg)
    time.sleep(0.5)

def turn_degrees(degrees, angular_speed=0.5, tolerance_rad=0.05):
    global current_yaw
    rate = rospy.Rate(20)
    initial_yaw = current_yaw
    target_yaw_rad = initial_yaw + math.radians(degrees)
    target_yaw_rad = math.atan2(math.sin(target_yaw_rad), math.cos(target_yaw_rad))

    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = angular_speed if degrees > 0 else -angular_speed
    while not rospy.is_shutdown():
        angle_diff = target_yaw_rad - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        if abs(angle_diff) <= tolerance_rad:
            rospy.loginfo("Reached target angle!")
            break

        twist_msg.angular.z = (angular_speed if degrees > 0 else -angular_speed) * min(1.0, abs(angle_diff) / math.radians(10))
        vel_pub.publish(twist_msg)
        rate.sleep()

    twist_msg.angular.z = 0.0
    vel_pub.publish(twist_msg)
    time.sleep(0.5)

def callback(data):
    global command
    text = data.data.lower()
    rospy.loginfo("set_command receive: %s", text)
    s_command = ["",""]
    if "forward" in text or "หน้า" in text:
        s_command[0] = "forward"
    elif "backward" in text or "หลัง" in text:
        s_command[0] = "backward"
    elif "left" in text or "ซ้าย" in text:
        s_command[0] = "left"
    elif "right" in text or "ขวา" in text:
        s_command[0] = "right"
    elif "delete" in text:
        if len(s_command) > 0:
            s_command.pop(-1)
            rospy.loginfo("delete last command")
    elif "command" in text:
        for c in command:
            if c[0] == "forward": move_distance(c[1])
            elif c[0] == "backward": move_distance(-c[1])
            elif c[0] == "left": turn_degrees(c[1])
            elif c[0] == "right": turn_degrees(-c[1])
        pub.publish("command stop")
        command = []
        return
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+", text)
    if numbers: s_command[1] = numbers[0]
    if s_command[0] != "" and s_command[1] != "":
        try:
            s_command[1] = float(s_command[1])
            pub.publish(str(s_command))
            command.append(s_command)
        except ValueError: pass

vel_pub = rospy.Publisher('final', Twist, queue_size=10)
pub = rospy.Publisher('recognized_speech_command', String, queue_size=10)
rospy.Subscriber('/set_command', String,callback)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
rospy.spin()