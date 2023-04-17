#! /usr/bin/env python

import rospy
import visualization_msgs.msg
import geometry_msgs
import tf
import roslib
import geometry_msgs.msg
import find_object_2d.msg._ObjectsStamped


listener = tf.TransformListener()
pub = rospy.Publisher('/hazards',visualization_msgs.msg.Marker)


marker = visualization_msgs.msg.Marker()
marker.header.frame_id(map)

def callback(self, data:find_object_2d.msg.ObjectsStamped):
    id = data.objects[0]
    objectID='/object'+id
    try:
        (trans,rot) = listener.lookupTransform('/map',objectID, rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    pose = geometry_msgs.msg.Pose(orientation=rot,position=trans)
    pub.publish(visualization_msgs.msg.Marker(id=id,action=0,pose=pose))
    

if __name__== '__main__':
    rospy.init_node('image_recognizer')
    while not rospy.is_shutdown():
        rospy.Subscriber("objectsStamped",find_object_2d.msg.ObjectsStamped,callback)