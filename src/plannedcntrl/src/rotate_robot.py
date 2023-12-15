#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def rotate_turtlebot():
    # Initialize the ROS node
    rospy.init_node('rotate_turtlebot', anonymous=True)

    # Create a publisher to control the TurtleBot's movement
    cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

    # Create a Twist message to send velocity commands
    twist_msg = Twist()

    # Set the linear velocity to 0
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0

    # Set the angular velocity for rotation (positive values rotate counterclockwise)
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = 0.3  # Adjust the angular velocity to control the rotation speed

    # Set the rate at which to publish the Twist message
    rate = rospy.Rate(10)  # 10 Hz

    # Set the duration for rotation (adjust as needed)
    rotation_duration = rospy.Duration(8.7*10/3)  # x seconds for 360-degree rotation

    # Get the current time
    start_time = rospy.Time.now()

    while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rotation_duration):
        # Publish the Twist message
        cmd_vel_pub.publish(twist_msg)

        # Sleep to maintain the desired publishing rate
        rate.sleep()

    # Stop the TurtleBot after completing the rotation
    twist_msg.angular.z = 0.0
    cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        rotate_turtlebot()
    except rospy.ROSInterruptException:
        pass