#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_msgs.msg import Float32MultiArray
from threading import Thread

import math

pub_ = None
signsId=[]
start = fa

#laser regions
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}

state_ = 0

state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

#define laser regions
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right': min(min(msg.ranges[0:143]),10),
        'fright': min(min(msg.ranges[144:287]),10),
        'front': min(min(msg.ranges[288:431]),10),
        'fleft': min(min(msg.ranges[432:575]),10),
        'left': min(min(msg.ranges[576:713]),10),
    }
    # rospy.loginfo(regions_)
    take_action()

def change_state(state):
    global state_,state_dict_
    if state is not state_:
        print('Wall follower - [%s] - [%s]' % (state,state_dict_[state]))
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    linear_y = 0
    angular_z = 0

    state_description = ''

    d = 0.3

    #no obstacles detected switch to state Find the wall
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    # obstacles in the front switch to state turn left
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    # obstacles in the right switch to state follow the wall
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    # obstacles in the left switch to state find the wall
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    # obstacles in the front and  right switch to state turn left
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    # obstacles in the front and left switch to state turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    # obstacles in the front and left and right switch to state turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    # obstacles in the left and right  switch to state find wall
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

#go stright and turn left
def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.15
    return msg
#turn left
def turn_left():
    msg = Twist()
    msg.angular.z = 0.15
    return msg
#go stright
def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.1
    return msg

def sign(sign):
    if sign[0]==0:
        start=True
    
    
def main():
    global pub_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    signs = rospy.Subscriber("/objects", Float32MultiArray, sign)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        while start:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
                pass
            else:
                rospy.logerr('Unknown state!')
            
            pub_.publish(msg)
            
            rate.sleep()

if __name__ == '__main__':
    main()