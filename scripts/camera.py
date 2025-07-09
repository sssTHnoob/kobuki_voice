#!/usr/bin/env python
import rospy
import subprocess
import os

gst_command = ["gst-launch-1.0"]

rospy.init_node('camera')
gst_command.extend([
	"v4l2src", "device=/dev/video0",
	"!", "video/x-raw,width=320,height=240,frame=15/1",
	"!", "nvvidconv",
	"!", "video/x-raw(memory:NVMM),format=NV12",
	"!", "nvv4l2h264enc",
	"!", "h264parse",
	"!", "rtph264pay", "config-interval=1",
	"!", "udpsink", "host=192.168.11.49", "port=5000"
])

process = subprocess.Popen(gst_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1)
for line in iter(process.stderr.readline, ''):
	rospy.logwarn("[GStreamer stderr] %s", line.strip())
for line in iter(process.stdout.readline, ''):
	rospy.logwarn("[GStreamer stdout] %s", line.strip())
process.wait()

rospy.loginfo("GStreamer pipeline finished.")
