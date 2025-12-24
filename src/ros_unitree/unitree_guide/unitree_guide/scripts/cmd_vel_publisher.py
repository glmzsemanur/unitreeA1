#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys

def main():
    rospy.init_node('cmd_vel_publisher')

    if len(sys.argv) < 5:
        print("Usage: rosrun unitree_guide cmd_vel_publisher.py <linear_x> <linear_y> <state> <angular_z>")
        sys.exit(1)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(20)  # 20 Hz

    msg = Twist()
    msg.linear.x = float(sys.argv[1])
    msg.linear.y = float(sys.argv[2])
    msg.linear.z = float(sys.argv[3])
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = float(sys.argv[4])

    for _ in range(10):
        pub.publish(msg)
        rate.sleep()
    rospy.loginfo(f"linear: [{float(sys.argv[1])}, {float(sys.argv[2])}], state: {float(sys.argv[3])}, angular.z: {float(sys.argv[4])}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

