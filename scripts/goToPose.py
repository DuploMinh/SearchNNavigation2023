#!/usr/bin/env python

# Implementation inspired by: https://github.com/danielsnider/follow_waypoints/blob/master/src/follow_waypoints/follow_waypoints.py

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Twist
)
from tf import TransformListener

def goToPose():
    pub_gotopose = rospy.Publisher('/move_base_simple/goal', PoseStamped)
    map_frame = rospy.get_param("~map_frame", 'map')
    robot_frame = rospy.get_param("~robot_frame", '/base_link')

    # Create target
    target_pose = PoseStamped()
    target_pose.header.frame_id = map_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = 1
    target_pose.pose.position.y = 1
    target_pose.pose.position.z = 0
    target_pose.pose.orientation.x = 0
    target_pose.pose.orientation.y = 0
    target_pose.pose.orientation.z = 0
    target_pose.pose.orientation.w = 1
    rospy.loginfo(target_pose)

    # Send goal
    # Need a little bit of a pause to let the node correctly launch
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.loginfo("Publishing")
    pub_gotopose.publish(target_pose)

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('rosbot_gotopose', anonymous=True)
        
        goToPose()
    except rospy.ROSInterruptException:
        pass
