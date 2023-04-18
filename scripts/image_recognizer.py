#! /usr/bin/env python3

import rospy
import visualization_msgs.msg
import geometry_msgs
import tf
import roslib
import geometry_msgs.msg
import find_object_2d.msg._ObjectsStamped



def callback(self, data:find_object_2d.msg.ObjectsStamped):
    id = int(data.objects.data[0])
    objectID='object'+id
    now = rospy.Time.now()
    try:
        listener.waitForTransform("/turtle2", "/carrot1", now, rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform('map',objectID, now)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    pose = geometry_msgs.msg.Pose(orientation=rot,position=trans)
    pub.publish(visualization_msgs.msg.Marker(id=id,action=0,pose=pose))
    
rospy.init_node('image_recognizer')
listener = tf.TransformListener()
while not rospy.is_shutdown():
    rospy.Subscriber("objectsStamped",find_object_2d.msg.ObjectsStamped,callback)
    rospy.spin()
pub = rospy.Publisher('/hazards',visualization_msgs.msg.Marker,queue_size=10)
marker = visualization_msgs.msg.Marker()

