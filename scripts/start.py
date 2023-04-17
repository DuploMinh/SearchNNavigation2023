#! /usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def callback(msg):
        if msg[0]==0:
           pass 
        
while not rospy.is_shutdown():
        rospy.Subscriber("objects", Float32MultiArray, callback)