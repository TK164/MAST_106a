#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def detect_objects(frame):
    # Use the MobileNet SSD model for object detection
    net = cv2.dnn.readNetFromCaffe('deploy.prototxt', 'mobilenet_iter_73000.caffemodel')

    # Resize input image to 300x300 pixels and normalize it
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)

    # Set the input to the model
    net.setInput(blob)

    # Forward pass and get the output predictions
    detections = net.forward()

    # Process the detections
    detected_objects = []
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.2:  # Confidence threshold
            class_id = int(detections[0, 0, i, 1])
            label = f'Class {class_id}'
            box = detections[0, 0, i, 3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
            (startX, startY, endX, endY) = box.astype("int")
            
            # Extract the region of interest (ROI) for color-based filtering
            roi_color = frame[startY:endY, startX:endX]

            # Convert BGR to HSV for color-based filtering
            hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)

            # Define the range of purple color in HSV
            lower_purple = np.array([130, 50, 50])
            upper_purple = np.array([170, 255, 255])

            # Threshold the HSV image to get only purple colors
            mask = cv2.inRange(hsv, lower_purple, upper_purple)

            # Bitwise-AND mask and original image
            cup_color_filtered = cv2.bitwise_and(roi_color, roi_color, mask=mask)

            # If there is a significant purple color in the ROI, consider it as a cup
            # print(mask)
            if cv2.countNonZero(mask) > 50:
                detected_objects.append({
                    'label': label,
                    'confidence': confidence,
                    'box': (startX, startY, endX, endY),
                    'color_filtered': cup_color_filtered
                })

    return detected_objects

def calculate_3d_position(detected_objects, depth_map):
    object_positions = []

    for obj in detected_objects:
        startX, startY, endX, endY = obj['box']

        # Get the depth values for the object region
        object_depth_values = depth_map[startY:endY, startX:endX]

        # Calculate the average depth value for the object
        average_depth = np.mean(object_depth_values)

        # Calculate the 3D position using the depth information
        x = (startX + endX) / 2
        y = (startY + endY) / 2
        z = average_depth

        object_positions.append({'label': obj['label'], 'position': (x, y, z)})

    return object_positions

def visualize_results(detected_objects, object_positions):
    # Visualize 2D bounding boxes and color-filtered images
    for obj in detected_objects:
        label = obj['label']
        box = obj['box']
        color_filtered = obj['color_filtered']

        cv2.rectangle(color_filtered, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)

        plt.figure()
        plt.imshow(cv2.cvtColor(color_filtered, cv2.COLOR_BGR2RGB))
        plt.title(f'Detected Object: {label}')
        plt.show()

    # Visualize 3D positions
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for obj in object_positions:
        position = obj['position']
        label = obj['label']
        ax.scatter(position[0], position[1], position[2], label=label)

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('3D Object Positions')
    ax.legend()
    plt.show()


def image_callback(color_image_msg, depth_image_msg):
    # Convert ROS color image message to OpenCV format
    color_image = CvBridge().imgmsg_to_cv2(color_image_msg, desired_encoding="bgr8")

    # Convert ROS depth image message to OpenCV format
    depth_image = CvBridge().imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

    # Detect purple cups
    detected_objects = detect_objects(color_image)

    # Calculate 3D positions
    object_positions = calculate_3d_position(detected_objects, depth_image)
    print(object_positions)
    
    #visualize_results(detected_objects, object_positions)

    # Publish object positions to a ROS topic
    for obj in object_positions:
        rospy.loginfo(f"{obj['label']} Position: {obj['position']}")
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = obj['position'][0]
        pose_msg.pose.position.y = obj['position'][1]
        pose_msg.pose.position.z = obj['position'][2]
        # Publish the 3D position to a topic
        # Assuming you have created a 'cup_positions' topic
        pub.publish(pose_msg)

if __name__ == "__main__":
    rospy.init_node('object_detection_node')

    # Subscribe to the color and depth image topics using ApproximateTimeSynchronizer
    color_sub = Subscriber('/camera/rgb/image_raw', Image)
    depth_sub = Subscriber('/camera/depth/image_raw', Image)
    
    ts = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1) 
    ts.registerCallback(image_callback)

    # Publish the 3D positions on a specific topic
    pub = rospy.Publisher('/cup_positions', PoseStamped, queue_size=10)

    rospy.spin()

