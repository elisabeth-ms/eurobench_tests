#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import LinkStates
import tf
def callback(data):
    rospy.loginfo(data)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("gazebo/link_states", LinkStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()