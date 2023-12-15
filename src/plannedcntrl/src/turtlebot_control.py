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
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point
from tf.transformations import quaternion_from_euler
from do_transform_pose import do_transform_pose
from trajectory import plan_curved_trajectory
from a_star import goal_trajectory
import matplotlib.pyplot as plt
from ccma import CCMA
import math
from std_msgs.msg import Float64MultiArray
import collections

#Define the method which contains the main functionality of the node.
def controller(waypoint, e_thresh):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - goal_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=50) ## TODO: what topic should we publish to? how?
  tfBuffer = tf2_ros.Buffer()## TODO: initialize a buffer
  tfListener = tf2_ros.TransformListener(tfBuffer) ## TODO: initialize a tf listener
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz
  # you can also use the rate to calculate your dt, but you don't have to
  N=10
  x_int = collections.deque(maxlen=N)
  y_int = collections.deque(maxlen=N)

  # All in the form [x, y]
  Kp = np.diag([.8, 1.2]) ## TODO: You may need to tune these values for your turtlebot
  Kd = np.diag([.2, 0.5]) ## TODO: You may need to tune these values for your turtlebot
  Ki = np.diag([0, .5]) ## TODO: You may need to tune these values for your turtlebot

  prev_time = rospy.get_time() ## TODO: initialize your time, what rospy function would be helpful here?
  integ = np.array([0.0, 0.0]) ## TODO: initialize an empty np array -- make sure to keep your sizes consistent
  derivative = np.array([]) ## TODO: initialize an empty np array 
  previous_error = np.array([0.0, 0.0]) ## TODO: initialize an empty np array 
  prev_vel = 0

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      #                                              target_frame, source_frame, current_time_in_ros
      trans_odom_to_base_link = tfBuffer.lookup_transform("base_footprint", "odom", rospy.Time()) ## TODO: create a transform between odom to base link

      (roll, pitch, baselink_yaw) = tf.transformations.euler_from_quaternion(
        [trans_odom_to_base_link.transform.rotation.x, trans_odom_to_base_link.transform.rotation.y,
            trans_odom_to_base_link.transform.rotation.z, trans_odom_to_base_link.transform.rotation.w])


      waypoint_trans = PoseStamped() ## TODO: initialize a PoseStamped
      waypoint_trans.pose.position.x = waypoint[0] ## TODO: what value would you use here?
      waypoint_trans.pose.position.y =waypoint[1] ## TODO: what value would you use here?
      waypoint_trans.pose.position.z = 0 ## TODO: what value would you use here?  # Assuming the waypoint is on the ground

      quat = quaternion_from_euler(0, 0, waypoint[2]) ## TODO: what would be the inputs to this function (there are 3)
      waypoint_trans.pose.orientation.x = quat[0] ## TODO: what value would you use here?
      waypoint_trans.pose.orientation.y = quat[1] ## TODO: what value would you use here?
      waypoint_trans.pose.orientation.z = quat[2] ## TODO: what value would you use here?
      waypoint_trans.pose.orientation.w = quat[3] ## TODO: what value would you use here?

      # Use the transform to compute the waypoint's pose in the base_link frame
      waypoint_in_base_link = do_transform_pose(waypoint_trans, trans_odom_to_base_link) ## TODO: what would be the inputs to this function (there are 2)
      (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [waypoint_in_base_link.pose.orientation.x, waypoint_in_base_link.pose.orientation.y,
            waypoint_in_base_link.pose.orientation.z, waypoint_in_base_link.pose.orientation.w])


      curr_time = rospy.get_time() ## TODO: get your time, what rospy function would be helpful here?

      # some debug output below
      print(f"Current: {trans_odom_to_base_link.transform.translation.x}, {trans_odom_to_base_link.transform.translation.y}, {baselink_yaw  }")
      print(f"Target: {waypoint}")
      #print(f"Waypoint in Baselink :{waypoint_in_base_link}")
      #print(f"Tranaformation: {trans_odom_to_base_link}")
      # Process trans to get your state error
      # Generate a control command to send to the robot
      x_error = waypoint_in_base_link.pose.position.x
      y_error = waypoint_in_base_link.pose.position.y
      theta_error = math.atan2(y_error, x_error)
      error = np.array([x_error, theta_error])## TODO: what are two values that we can use for this np.array, and what are the dimensions
      print("error:", error)
      
      # proportional term
      proportional = np.dot(Kp, error).squeeze()
      
      # integral term
      dt = curr_time - prev_time ## TODO: quick operation to determine
      x_int.append(error[0]*dt)
      y_int.append(error[1]*dt)

      integ = np.array([sum(x_int), sum(y_int)]) ## TODO: integral is summing up error over time, so what would we expect to add on to our integral term tracker here?
      integral = np.dot(Ki, integ).squeeze()

      # dervative term
      error_deriv = (error - previous_error)/dt ## TODO: quick operation to determine dt
      derivative = np.dot(Kd, error_deriv).squeeze()

      msg = Twist()
      vel = proportional[0] + derivative[0] + integral[0]
      if vel - prev_vel > .2:
        vel = prev_vel + .2
      msg.linear.x  = vel
      msg.angular.z = proportional[1] + derivative[1] + integral[1] 
      print("Velocity:\n", msg)

      control_command = msg

      #################################### end your code ###############

      previous_error = error ## TODO
      prev_time = curr_time ## TODO
      pub.publish(control_command)

      if np.abs(error[0]) < e_thresh[0] and (np.abs(error[1]) < e_thresh[1] or np.abs(y_error) < e_thresh[1]) : ##TODO: what is our stopping condition/how do we know to go to the next waypoint?
      # if np.linalg.norm(error) < .1:
        print("Moving to next waypoint in trajectory")
        return

    except Exception as e:
      print(e)
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()


def planning_callback(waypoints):
  try:
    #controller((0, 0, 0))
    for w in waypoints[:-1]:
      controller(w, (.015, .1))
    controller(waypoints[-1], (.015, .1))
      

  except rospy.ROSInterruptException as e:
    print("Exception thrown in planning callback: " + e)
    pass
      

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.x
  rospy.init_node('turtlebot_controller', anonymous=True)
  
  show_animation = sys.argv[3]
  
  # GOAL_X = []
  # GOAL_Y = []
  # msg = rospy.wait_for_message('/ar_tag_16', TransformStamped)
  # print("Got 16")
  # GOAL_X.append(msg.transform.translation.x)
  # GOAL_Y.append(msg.transform.translation.y)
  
  # msg = rospy.wait_for_message('/ar_tag_17', TransformStamped)
  # print("Got 17")
  # GOAL_X.append(msg.transform.translation.x)
  # GOAL_Y.append(msg.transform.translation.y)
  
  tfBuffer = tf2_ros.Buffer()## TODO: initialize a buffer
  tfListener = tf2_ros.TransformListener(tfBuffer) ## TODO: initialize a tf listener
  trans = tfBuffer.lookup_transform("odom", "base_footprint", rospy.Time(), rospy.Duration(1.0)) ## TODO: create a transform between base link and odom

    
  
  print(f"Current: {trans.transform.translation.x}, {trans.transform.translation.y}")
  
  ROBOT_X = trans.transform.translation.x
  ROBOT_Y = trans.transform.translation.y
  # message = rospy.wait_for_message('/pink_point', Point)
  GOAL_X = float(sys.argv[1])
  GOAL_Y = float(sys.argv[2])
  print(GOAL_X, GOAL_Y)
  message = list(rospy.wait_for_message('/map_data', Float64MultiArray).data)
  message.extend(list(rospy.get_param('og_map')))
  map_data = np.array(message).reshape(-1, 2)
  
  map_data = np.delete(map_data, np.where((np.abs(map_data[:, 0]-GOAL_Y) <= .5) & (np.abs(map_data[:, 1]-GOAL_X) <= .5))[0], axis=0)
  waypoints_to_goal = goal_trajectory(ROBOT_X, ROBOT_Y, GOAL_X, GOAL_Y, True, map_data)
  
  rx_s = [point[0] for point in waypoints_to_goal]
  ry_s = [point[1] for point in waypoints_to_goal]

  if show_animation:  # pragma: no cover
      plt.plot(rx_s, ry_s, ".r")
      # plt.plot(rx, ry, ".b")
      plt.plot(GOAL_X, GOAL_Y, "og")
      plt.pause(0.001)
      plt.show()
  
  planning_callback(waypoints_to_goal)
  
  # trans = tfBuffer.lookup_transform("odom", "base_footprint", rospy.Time(), rospy.Duration(1.0)) ## TODO: create a transform between base link and odom
  # ROBOT_X = trans.transform.translation.x
  # ROBOT_Y = trans.transform.translation.y
  
  # waypoints_to_goal = goal_trajectory(ROBOT_X, ROBOT_Y, GOAL_X[0], GOAL_Y[0], True, map_data)
  
  # rx_s = [point[0] for point in waypoints_to_goal]
  # ry_s = [point[1] for point in waypoints_to_goal]

  # if show_animation:  # pragma: no cover
  #     plt.plot(rx_s, ry_s, ".r")
  #     # plt.plot(rx, ry, ".b")
  #     plt.plot(GOAL_X, GOAL_Y, "og")
  #     plt.pause(0.001)
  #     plt.show()
  
  # planning_callback(waypoints_to_goal)

  #rospy.Subscriber("goal_point", Point, planning_callback, queue_size=1) ## TODO: what are we subscribing to here?
  
  # rospy.spin()
