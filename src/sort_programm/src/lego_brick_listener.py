#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.point)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lego_brick_listener', anonymous=True)

    rospy.Subscriber("/lego_brick_coords", PointStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

