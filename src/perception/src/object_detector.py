#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import time
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')


class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        self.color_image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.color_image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_image_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub_pink = rospy.Publisher("pink_point", Point, queue_size=10)
        self.image_pub_pink = rospy.Publisher('detected_pink', Image, queue_size=10)
        
        self.point_pub_yellow = rospy.Publisher("yellow_point", Point, queue_size=10)
        self.image_pub_yellow = rospy.Publisher('detected_yellow', Image, queue_size=10)
        
        self.point_pub_green = rospy.Publisher("green_point", Point, queue_size=10)
        self.image_pub_green = rospy.Publisher('detected_green', Image, queue_size=10)

        rospy.spin()

    def camera_info_callback(self, msg):
        # TODO: Extract the intrinsic parameters from the CameraInfo message (look this message type up online)
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth):
        # TODO: Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = (u - self.cx)*depth/self.fx
        Y = (v - self.cy)*depth/self.fy
        Z = depth
        return X, Y, Z

    def color_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8 format)
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            
            # If we have both color and depth images, process them
            if self.cv_depth_image is not None:
                self.process_images()

        except Exception as e:   
            print("Error:", e)

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("That one")
            print("Error:", e)

    def process_images(self):
        # Convert the color image to HSV color space
        hsv = cv2.cvtColor(self.cv_color_image, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(hsv, (25, 25), 2)
        mean_center_row_hsv_val = np.mean(hsv[len(hsv)//2], axis=0)
        ### print("Current mean values at center row of image: ", mean_center_row_hsv_val)
        
        ## Orange: 15 195 175 --> Blends with cardboard :(
        #lower_hsv = np.array([5, 190, 170])
        #upper_hsv = np.array([30, 200, 180])
        
        ## Blue: 100 85 128 --> BAD blue is everyrewhe
        #lower_hsv = np.array([90, 75, 118])
        #upper_hsv = np.array([110, 95, 138])
        
        #Green: 60 220 109 --> Turn off blur
        lower_green = np.array([40, 160, 100])
        upper_green = np.array([80, 250, 220])
        
        #Yellow: 30 215 130 ,31 212, 162 // 37 200 179, 23 143 159
        lower_yellow = np.array([20, 180, 110])
        upper_yellow = np.array([40, 230, 190])

        # Pink:
        ## lower_hsv = np.array([160, 180, 160])
        ## upper_hsv = np.array([180, 200, 180])
        lower_pink = np.array([155, 150, 150])
        upper_pink = np.array([185, 200, 200])
        
        self.get_coords(lower_pink, upper_pink, blur, "pink", 1000)
        self.get_coords(lower_yellow, upper_yellow, blur, "yellow", 50)
        self.get_coords(lower_green, upper_green, blur, "green", 1500)
    
    def get_coords(self, lower_hsv, upper_hsv, img, color, pixel_thresh):
        point_pub = ""
        image_pub = ""
        if color == "pink":
    	    point_pub = self.point_pub_pink
    	    image_pub = self.image_pub_pink
        elif color == "yellow":
    	    point_pub = self.point_pub_yellow
    	    image_pub = self.image_pub_yellow
        elif color == "green":
    	    point_pub = self.point_pub_green
    	    image_pub = self.image_pub_green

        # TODO: Threshold the image to get only cup colors
        # HINT: Lookup cv2.inRange() or np.where()
        #mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        mask = cv2.inRange(img, lower_hsv, upper_hsv)
        ### if color == "green":
            ### print("Num green pixels: " + str(len(np.nonzero(mask)[0])))

        # TODO: Get the coordinates of the cup points on the mask
        # HINT: Lookup np.nonzero()
        y_coords, x_coords = np.nonzero(mask)

        # If there are not enough detected points, exit
        if len(x_coords) <= pixel_thresh or len(y_coords) <= pixel_thresh:
            ### print("No points detected. Is your color filter wrong?")
            return

        # Calculate the center of the detected region by 
        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))

        # Fetch the depth value at the center
        depth = self.cv_depth_image[center_y, center_x]

        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(center_x, center_y, depth)
            camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
            # Convert from mm to m
            camera_link_x /= 1000
            camera_link_y /= 1000
            camera_link_z /= 1000

            # Convert the (X, Y, Z) coordinates from camera frame to odom frame
            try:
                self.tf_listener.waitForTransform("/odom", "/camera_link", rospy.Time(), rospy.Duration(10.0))
                point_odom = self.tf_listener.transformPoint("/odom", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                X_odom, Y_odom, Z_odom = point_odom.point.x, point_odom.point.y, point_odom.point.z
                ### print(color + ":")
                ### print("Real-world coordinates in odom frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(X_odom, Y_odom, Z_odom))

                if X_odom < 0.001 and X_odom > -0.001:
                    print("Erroneous goal point, not publishing - Is the cup too close to the camera?")
                else:
                    ### print("Publishing goal point: ", X_odom, Y_odom, Z_odom)
                    # Publish the transformed point
                    point_pub.publish(Point(X_odom, Y_odom, Z_odom))

                    # Overlay cup points on color image for visualization
                    cup_img = self.cv_color_image.copy()
                    cup_img[y_coords, x_coords] = [0, 0, 255]  # Highlight cup points in red
                    cv2.circle(cup_img, (center_x, center_y), 5, [0, 255, 0], -1)  # Draw green circle at center
                    
                    # Convert to ROS Image message and publish
                    ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
                    image_pub.publish(ros_image)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + e)
                return

if __name__ == '__main__':
    ObjectDetector()
