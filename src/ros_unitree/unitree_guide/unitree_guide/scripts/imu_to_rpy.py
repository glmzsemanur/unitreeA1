#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

# Publisher for RPY
rpy_pub = None

def imu_callback(msg):
    # Convert quaternion to RPY
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    # Publish as Vector3
    rpy_msg = Vector3()
    rpy_msg.x = roll
    rpy_msg.y = pitch
    rpy_msg.z = yaw
    rpy_pub.publish(rpy_msg)

def main():
    global rpy_pub
    rospy.init_node('imu_to_rpy')
    
    # Publisher for RPY
    rpy_pub = rospy.Publisher('/trunk_imu_rpy', Vector3, queue_size=10)
    
    # Subscribe to your IMU
    rospy.Subscriber('/trunk_imu', Imu, imu_callback)
    
    rospy.loginfo("IMU to RPY node started")
    rospy.spin()

if __name__ == '__main__':
    main()
