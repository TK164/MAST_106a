#!/usr/bin/env python3
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import tf
import sys
import numpy as np
from geometry_msgs.msg import TransformStamped

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.x
  rospy.init_node('get_ar_coords', anonymous=True)
  
  ar_pub_16 = rospy.Publisher("ar_tag_16", TransformStamped, queue_size=10)
  ar_pub_17 = rospy.Publisher('ar_tag_17', TransformStamped, queue_size=10)
  
  tfBuffer = tf2_ros.Buffer()## TODO: initialize a buffer
  tfListener = tf2_ros.TransformListener(tfBuffer) ## TODO: initialize a tf listener
  
  while not rospy.is_shutdown():
    try:
      trans_ar_16 = tfBuffer.lookup_transform('odom','ar_marker_16', rospy.Time(), rospy.Duration(1.0))
      trans_base_link = tfBuffer.lookup_transform('odom','base_footprint', rospy.Time(), rospy.Duration(1.0))
      tag_vec = np.array([trans_ar_16.transform.translation.x, trans_ar_16.transform.translation.y])
      robot_vec = np.array([trans_base_link.transform.translation.x, trans_base_link.transform.translation.y])
      unit_vec = (tag_vec-robot_vec)/np.linalg.norm(tag_vec-robot_vec)
      vec = tag_vec-unit_vec*.1

      trans_ar_16.transform.translation.x = vec[0]
      trans_ar_16.transform.translation.y = vec[1]
      ar_pub_16.publish(trans_ar_16)
      print(trans_ar_16)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print("TF Error in Turtlebot Controller: " + str(e))
      pass
    
    try:
      trans_ar_17 = tfBuffer.lookup_transform('odom','ar_marker_17', rospy.Time(), rospy.Duration(1.0))
      trans_base_link = tfBuffer.lookup_transform('odom','base_footprint', rospy.Time(), rospy.Duration(1.0))
      tag_vec = np.array([trans_ar_17.transform.translation.x, trans_ar_17.transform.translation.y])
      robot_vec = np.array([trans_base_link.transform.translation.x, trans_base_link.transform.translation.y])
      unit_vec = (tag_vec-robot_vec)/np.linalg.norm(tag_vec-robot_vec)
      vec = tag_vec-unit_vec*.1

      trans_ar_17.transform.translation.x = vec[0]
      trans_ar_17.transform.translation.y = vec[1]

      ar_pub_17.publish(trans_ar_17)
      print(trans_ar_17)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print("TF Error in Turtlebot Controller: " + str(e))
      pass
     
    # Use our rate object to sleep until it is time to publish again
