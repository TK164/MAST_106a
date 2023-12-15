#!/usr/bin/env python3
"""
SCRIPT STEPS:

Step 1: BUILD ENV MAP
* initialize map builder launch - rosrun mapping demo.launch
* hardcoded commands to drive forward to each room and slowly spin, then return to ODOM
* save map data, goal locations data, publish ONCE to A star path planner

Step 2: PATH to CUP
* Locate first cup using camera / basic spin until HSV Object Detection sees cup center
* Use lab 8 path planning and PID control to move to cup / "grab" with velcro

Step 3: PATH to GOAL
* Use A* path planning from current robot pos to goal pos to get waypoints
* turn robot to face angle of first waypoints towards 1 goal
* execute PID control to goal 

STEP 4: PROMPT USER INPUT
* with cup delivered, wait for user to remove cup, input if correct goal was reached
* If "correct", return to odom, repeat from step 2
* If "incorrect", path plan to other goal from current location, then repeat from step 2 to other location
"""
import rospy
from geometry_msgs.msg import Point, PoseStamped,TransformStamped
import subprocess
from std_msgs.msg import Float64MultiArray


detected_point = None
imovable_objects = []
goal_coords = []
pink_goal = None
green_goal = None
curr_color = None


def start_ros_script(package_name, script_name):
    # Run a ROS script as a subprocess
    return subprocess.Popen(['rosrun', package_name, script_name + '.py'])

def run_ros_script(package_name, script_name, x, y):
    # Run a ROS script as a subprocess
    subprocess.call(['rosrun', package_name, script_name + '.py', x, y, "True"])

def callback_green(data):
    #rospy.loginfo("Received X point for the green cup: {}".format(data.x))
    # Set the parameter to True when a message is received for the green cup
    global detected_point
    global curr_color
    rospy.set_param('green_cup_received', True)
    detected_point = data
    curr_color = "green"

def callback_pink(data):
    #rospy.loginfo("Received X point for the pink cup: {}".format(data.x))
    # Set the parameter to True when a message is received for the pink cup
    global detected_point
    global curr_color
    rospy.set_param('pink_cup_received', True)
    detected_point = data
    curr_color = "pink"

def check_topic_messages():
    # Topics for different colored cups
    #topic_yellow = '/yellow_point'
    topic_green = '/green_point'
    topic_pink = '/pink_point'

    # Define callback functions for each topic
    #rospy.Subscriber(topic_yellow, Point, callback_yellow)
    rospy.Subscriber(topic_green, Point, callback_green)
    rospy.Subscriber(topic_pink, Point, callback_pink)

    # Wait for messages on any of the topics
    rospy.loginfo("Waiting for cup poses...")
    while not rospy.is_shutdown():
        rospy.sleep(1)  # Adjust sleep duration as needed
        if rospy.core.is_initialized():
            # Check if messages have been received on any of the topics
            if rospy.get_param('green_cup_received') or rospy.get_param('pink_cup_received'): # rospy.get_param('yellow_cup_received')
                return True

    return False

def step2_cup():
    global detected_point
    global detected_cup
    #intialize script to publish HSV cup detection:

    #hard code spin until cup detected
    search_for_cup = start_ros_script('plannedcntrl', 'rotate_robot')

    # Wait until cup is detected -> wait until a message is received on one of the topics
    check_topic_messages()
    
    search_for_cup.kill()
    search_for_cup.wait()
    print("search killed")
    
    print(curr_color)
    if curr_color == "pink":
        detected_cup = rospy.wait_for_message('/pink_point', Point)
    else:
        detected_cup = rospy.wait_for_message('/green_point', Point)
    print(detected_cup)
    # detected_cup.kill()
    # detected_cup.wait()
    # print("object found, detected killed")

    # initialize path planning from lab 8 to get to cup:
    # run_ros_script('', '')
    subprocess.call(['rosrun', 'plannedcntrl', 'turtlebot_control' + '.py', str(detected_point.x), str(detected_point.y), "True"])

def step3_goal(point):
    # call A* planner, turn robot to face waypoints, then execute with PID
    run_ros_script('plannedcntrl', 'turtlebot_control', str(point[0]), str(point[1]))

def master_demo():

    #STEP 1: done outside of this function
    # #run_ros_script('mapping', 'demo.py')  would be run manually with teleop on control
    # would also run AR tag tracking to get goal location
    goal16 = None
    goal17 = None
    while (not goal16) or (not goal17):
        try:
            if not goal16:
                goal16 = rospy.wait_for_message('/ar_tag_16', TransformStamped, timeout=1)
        except Exception as e:
            print("AR tag 16 not found")
        try:
            if not goal17:
                goal17 = rospy.wait_for_message('/ar_tag_17', TransformStamped, timeout=1)
        except Exception as e:
            print("AR tag 17 not found")

    print(goal16, goal17)
    
    global goal_coords
    goal_coords = [(goal16.transform.translation.x, goal16.transform.translation.y), (goal17.transform.translation.x, goal17.transform.translation.y)]

    input('Press any key to get map:')
    message = rospy.wait_for_message('/map_data', Float64MultiArray)
    rospy.set_param('og_map', message.data)
    input("Press any key to start:")

    #rospy.set_param('yellow_cup_received', False)
    rospy.set_param('green_cup_received', False)
    rospy.set_param('pink_cup_received', False)

    

    # rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint camera_link 100
    # start_tf = subprocess.Popen(['rosrun', 'tf', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'base_footprint', 'camera_link', '100'])
    detected_cup = start_ros_script('perception', 'object_detector')
    #STEP 2:
    step2_cup()
    # start_tf.kill()
    detected_cup.kill()
    #STEP 3:
    step3_goal(goal_coords[0])

    correct = input("Was this the correct goal?: ")
    if correct == "y":
        if curr_color == "pink":
            pink_goal = 0
            green_goal = 1
        else:
            pink_goal = 1
            green_goal = 0
    else:
        if curr_color == "pink":
            pink_goal = 1
            green_goal = 0
        else:
            pink_goal = 0
            green_goal = 1
    
    for _ in range(3):
        step3_goal([0,0])
        #rospy.set_param('yellow_cup_received', False)
        rospy.set_param('pink_cup_received', False)
        rospy.set_param('green_cup_received', False)

        # start_tf = subprocess.Popen(['rosrun', 'tf', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'base_footprint', 'camera_link', '100'])
        detected_cup = start_ros_script('perception', 'object_detector')
        step2_cup()
        # start_tf.kill()
        # start_tf.wait()
        detected_cup.kill()
        detected_cup.wait()
        if curr_color == "pink":
            step3_goal(goal_coords[pink_goal])
        else:
            step3_goal(goal_coords[green_goal])




    
    
    # ... other steps ...

if __name__ == '__main__':
    rospy.init_node('robot_master_node', anonymous=True)

    #rospy.set_param('yellow_cup_received', False)
    rospy.set_param('green_cup_received', False)
    rospy.set_param('pink_cup_received', False)
    
    master_demo()