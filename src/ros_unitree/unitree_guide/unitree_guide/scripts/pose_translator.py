#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseTranslator:
    def __init__(self):
        self.pub = rospy.Publisher(
            '/robot_pose',
            PoseWithCovarianceStamped,
            queue_size=200
        )
        rospy.Subscriber(
            '/odometry/filtered',
            Odometry,
            self.callback,
            queue_size=200
        )

    def callback(self, msg):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose
        self.pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('pose_translator')
    PoseTranslator()
    rospy.spin()

