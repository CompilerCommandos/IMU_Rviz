#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

# Callback to process incoming IMU messages
def imu_callback(msg):
    rospy.loginfo("Received IMU data:")
    rospy.loginfo("Linear Acceleration -> x: {:.2f}, y: {:.2f}, z: {:.2f}".format(
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z
    ))
    rospy.loginfo("Angular Velocity -> x: {:.2f}, y: {:.2f}, z: {:.2f}".format(
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z
    ))
    rospy.loginfo("Orientation -> x: {:.2f}, y: {:.2f}, z: {:.2f}, w: {:.2f}".format(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ))

# Main function to initialize the ROS node and subscribe to the IMU topic
def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)

    # Subscribe to the topic published by the Arduino code
    rospy.Subscriber("/imu/data", Imu, imu_callback)

    # Keep the node running to process incoming messages
    rospy.spin()

# Entry point
if __name__ == '__main__':
    try:
        imu_listener()
    except rospy.ROSInterruptException:
        pass

