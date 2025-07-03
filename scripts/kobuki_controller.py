#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class KobukiVoiceController:
    def __init__(self):
        rospy.init_node('kobuki_voice_controller', anonymous=True)

        # Publisher for Kobuki's velocity commands
        # The default topic for Kobuki's velocity commands is '/cmd_vel'
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to your speech recognition topic
        rospy.Subscriber('/recognized_speech', String, self.speech_callback)

        self.rate = rospy.Rate(10)  # 10 Hz control loop

        rospy.loginfo("Kobuki Voice Controller node started. Waiting for speech commands...")

    def speech_callback(self, data):
        recognized_text = data.data.lower()  # Convert to lowercase for easier comparison
        rospy.loginfo("Received command: '%s'", recognized_text)

        twist_msg = Twist()  # Initialize a new Twist message for each command

        # --- Command Interpretation Logic ---
        if "move forward" in recognized_text or "go forward" in recognized_text:
            rospy.loginfo("Moving Kobuki forward...")
            twist_msg.linear.x = 1  # meters/second
            twist_msg.angular.z = 0.0
            self.publish_and_hold(twist_msg, duration=2.0)  # Move for 2 seconds

        elif "move backward" in recognized_text or "go backward" in recognized_text:
            rospy.loginfo("Moving Kobuki backward...")
            twist_msg.linear.x = -0.1  # meters/second
            twist_msg.angular.z = 0.0
            self.publish_and_hold(twist_msg, duration=2.0)

        elif "turn left" in recognized_text:
            rospy.loginfo("Turning Kobuki left...")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5  # radians/second
            self.publish_and_hold(twist_msg, duration=2.0)

        elif "turn right" in recognized_text:
            rospy.loginfo("Turning Kobuki right...")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -0.5  # radians/second
            self.publish_and_hold(twist_msg, duration=2.0)

        elif "stop" in recognized_text or "halt" in recognized_text:
            rospy.loginfo("Stopping Kobuki.")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.publish_and_hold(twist_msg, duration=0.5)  # Send stop command for a short duration

        else:
            rospy.loginfo("Unrecognized command. Did you say: '%s'?", recognized_text)

    def publish_and_hold(self, twist_msg, duration):
        # This function publishes the command repeatedly for 'duration' seconds
        # and then sends a stop command. This is crucial because Kobuki (and most robots)
        # require continuous velocity commands.
        start_time = time.time()
        while (time.time() - start_time) < duration and not rospy.is_shutdown():
            self.cmd_vel_publisher.publish(twist_msg)
            self.rate.sleep()

        # Always send a stop command after the action
        self.cmd_vel_publisher.publish(Twist())  # Sends an empty (zero velocity) Twist message
        rospy.loginfo("Command execution complete. Kobuki stopped.")

    def run(self):
        rospy.spin()  # Keeps the node alive and processes callbacks


if __name__ == '__main__':
    try:
        controller = KobukiVoiceController()
        controller.run()
    except rospy.ROSInterruptException:
        pass