#!/usr/bin/env python

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Twist
)
from tf import TransformListener
from tf.transformations import euler_from_quaternion

def drive():
    # Drive publisher
    pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd = Twist()
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0

    # Transform listener
    tfl = TransformListener()

    # Paramaters
    # Clip speed for safety
    max_speed = rospy.get_param("~speed", 2.0)
    # Execute for a max time for safety
    max_duration = rospy.get_param("~duration", 10)
    # Target angle (in radians)
    set_point = rospy.get_param("~set_point", -1.5)
    # From frame
    frame_robot = "/base_link"
    frame_odom = rospy.get_param("~frame_odom", "/odom")

    # PID elements
    gain_p = rospy.get_param("~gain_p", 1.5)
    error = 0

    # Periodically publish the drive command to ensure drive continues
    rate = rospy.Rate(10)
    t_start = rospy.Time.now()
    stop = False
    while (not stop) and (not rospy.is_shutdown()):
        # Transform into odom frame
        t_tf = rospy.Time.now()# - rospy.Duration(0.1)
        tfl.waitForTransform(frame_odom, frame_robot, t_tf, rospy.Duration(2.0))

        pose_zero = PoseStamped()
        pose_zero.header.frame_id = frame_robot
        pose_zero.header.stamp = t_tf
        pose_odom = tfl.transformPose(frame_odom, pose_zero)
        rospy.logdebug(pose_odom)

        # need to convert geometry_msgs to transform quaternion format
        # array, brack notation in euler call is important
        g_quat = pose_odom.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([g_quat.x, g_quat.y, g_quat.z, g_quat.w])

        # Compute PID
        error = set_point - yaw
        rot_cmd = gain_p * error

        # Clip
        rot_cmd = min(rot_cmd, max_speed)
        rot_cmd = max(rot_cmd, -max_speed)

        cmd.angular.z = rot_cmd

        # Publish Command
        rospy.loginfo("Error: " + str(error))
        rospy.loginfo(cmd)
        pub_drive.publish(cmd)
        rate.sleep()
        
        # Check time
        t_now = rospy.Time.now()
        duration = t_now - t_start
        if t_now - t_start > rospy.Duration(max_duration):
            stop = True

    # Ensure a stop command is sent
    cmd.linear.x = 0
    cmd.angular.z = 0
    pub_drive.publish(cmd)

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('rosbot_pid', anonymous=True)
        drive()
    except rospy.ROSInterruptException:
        pass
