################################################################################
#
# OccupancyGrid2d class listens for LaserScans and builds an occupancy grid.
#
################################################################################

import rospy
import tf2_ros
import tf
import sys

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float64MultiArray
import numpy as np

class OccupancyGrid2d(object):
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up the map.
        self._map = np.zeros((self._x_num, self._y_num))

        self._initialized = True
        return True

    def LoadParameters(self):
        # Random downsampling fraction, i.e. only keep this fraction of rays.
        if not rospy.has_param("~random_downsample"):
            return False
        self._random_downsample = rospy.get_param("~random_downsample")

        # Dimensions and bounds.
        # TODO! You'll need to set values for class variables called:
        # -- self._x_num
        if not rospy.has_param("~x/num"):
            return False
        self._x_num = rospy.get_param("~x/num")
        # -- self._x_min
        if not rospy.has_param("~x/min"):
            return False
        self._x_min = rospy.get_param("~x/min")
        # -- self._x_max
        if not rospy.has_param("~x/max"):
            return False
        self._x_max = rospy.get_param("~x/max")
        # -- self._x_res # The resolution in x. Note: This isn't a ROS parameter. What will you do instead?
        self._x_res = (self._x_max - self._x_min)/self._x_num 
        # -- self._y_num
        if not rospy.has_param("~y/num"):
            return False
        self._y_num = rospy.get_param("~y/num")
        # -- self._y_min
        if not rospy.has_param("~y/min"):
            return False
        self._y_min = rospy.get_param("~y/min")
        # -- self._y_max
        if not rospy.has_param("~y/max"):
            return False
        self._y_max = rospy.get_param("~y/max")
        # -- self._y_res # The resolution in y. Note: This isn't a ROS parameter. What will you do instead?
        self._y_res = (self._y_max - self._y_min)/self._y_num

        # Update parameters.
        if not rospy.has_param("~update/occupied"):
            return False
        self._occupied_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied"))

        if not rospy.has_param("~update/occupied_threshold"):
            return False
        self._occupied_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied_threshold"))

        if not rospy.has_param("~update/free"):
            return False
        self._free_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free"))

        if not rospy.has_param("~update/free_threshold"):
            return False
        self._free_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free_threshold"))

        # Topics.
        # TODO! You'll need to set values for class variables called:
        # self._sensor_topic
        if not rospy.has_param("~topics/sensor"):
            return False
        self._sensor_topic = rospy.get_param("~topics/sensor")
        #self._vis_topic
        if not rospy.has_param("~topics/vis"):
            return False
        self._vis_topic = rospy.get_param("~topics/vis")

        # Frames.
        # TODO! You'll need to set values for class variables called:
        #self._sensor_frame
        if not rospy.has_param("~frames/sensor"):
            return False
        self._sensor_frame = rospy.get_param("~frames/sensor")
        #self._fixed_frame
        if not rospy.has_param("~frames/fixed"):
            return False
        self._fixed_frame = rospy.get_param("~frames/fixed")

        return True

    def RegisterCallbacks(self):
        # Subscriber.
        self._sensor_sub = rospy.Subscriber(self._sensor_topic,
                                            LaserScan,
                                            self.SensorCallback,
                                            queue_size=1)

        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        OccupancyGrid,
                                        queue_size=10)
                                        
        self._map_pub = rospy.Publisher("map_data",
                                        Float64MultiArray,
                                        queue_size=10)                                
                                        

        return True

    # Callback to process sensor measurements.
    def SensorCallback(self, msg):
        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return

        # Get our current pose from TF.
        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Writes an error message to the ROS log but does not raise an exception
            rospy.logerr("%s: Could not extract pose from TF.", self._name)
            return

        # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
        # assuming that the turtlebot is on the ground plane.
        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y
        if abs(pose.transform.translation.z) > 0.05:
            rospy.logwarn("%s: Turtlebot is not on ground plane.", self._name)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            rospy.logwarn("%s: Turtlebot roll/pitch is too large.", self._name)

        # Loop over all ranges in the LaserScan.
        for idx, r in enumerate(msg.ranges):
            # Randomly throw out some rays to speed this up.
            if np.random.rand() > self._random_downsample:
                continue
            elif np.isnan(r):
                continue

            # Get angle of this ray in fixed frame.
            # TODO!
            angle = msg.angle_min + idx*msg.angle_increment + yaw

            # Throw out this point if it is too close or too far away.
            if r > msg.range_max:
                rospy.logwarn("%s: Range %f > %f was too large.",
                              self._name, r, msg.range_max)
                continue
            if r < msg.range_min:
                rospy.logwarn("%s: Range %f < %f was too small.",
                              self._name, r, msg.range_min)
                continue

            # Walk along this ray from the scan point to the sensor.
            # Update log-odds at each voxel along the way.
            # Only update each voxel once. 
            # The occupancy grid is stored in self._map
            # TODO!
            # end_x = sensor_x + r*np.cos(angle)
            # end_y = sensor_y + r*np.sin(angle)

            # if (np.abs(end_x - sensor_x) > np.abs(end_y - sensor_y)):
            #     x_arr = np.arange(end_x, sensor_x, self._x_res)
            #     y_arr = np.arange(end_y, sensor_y, self._x_res*np.tan(angle))
            # else:
            #     x_arr = np.arange(end_x, sensor_x, self._y_res/np.tan(angle))
            #     y_arr = np.arange(end_y, sensor_y, self._y_res)
            prev = (None, None)
            for i in np.arange(r, 0, -min(self._x_res, self._y_res)):
                end_x = sensor_x + i*np.cos(angle)
                end_y = sensor_y + i*np.sin(angle)
                curr = self.PointToVoxel(end_x, end_y)
                if prev != curr:
                    if i == r:
                        # delta occ
                        self._map[curr[0]][curr[1]] = min(self._occupied_threshold, self._map[curr[0]][curr[1]] + self._occupied_update)
                    else:
                        # delta free
                        self._map[curr[0]][curr[1]] = max(self._free_threshold, self._map[curr[0]][curr[1]] + self._free_update)
                prev = curr

        # Visualize.
        self.Visualize(pose)

    # Convert (x, y) coordinates in fixed frame to grid coordinates.
    def PointToVoxel(self, x, y):
        grid_y = int((x - self._x_min) / self._x_res)
        grid_x = int((y - self._y_min) / self._y_res)

        return (grid_x, grid_y)

    # Get the center point (x, y) corresponding to the given voxel.
    def VoxelCenter(self, ii, jj):
        center_x = self._x_min + (0.5 + ii) * self._x_res
        center_y = self._y_min + (0.5 + jj) * self._y_res

        return (center_x, center_y)

    # Convert between probabity and log-odds.
    def ProbabilityToLogOdds(self, p):
        return np.log(p / (1.0 - p))

    def LogOddsToProbability(self, l):
        return 1.0 / (1.0 + np.exp(-l))

    # Colormap to take log odds at a voxel to a RGBA color.
    def Colormap(self, ii, jj):
        p = self.LogOddsToProbability(self._map[ii, jj])

        c = ColorRGBA()
        c.r = p
        c.g = 0.1
        c.b = 1.0 - p
        c.a = 0.75
        return c

    # Visualize the map as a collection of flat cubes instead of
    # as a built-in OccupancyGrid message, since that gives us more
    # flexibility for things like color maps and stuff.
    # See http://wiki.ros.org/rviz/DisplayTypes/Marker for a brief tutorial.
    def Visualize(self, pose):
        m = OccupancyGrid()
        m.header.stamp, m.header.frame_id = rospy.Time.now(), self._fixed_frame
        #m.info.origin.position.x, m.info.origin.position.y = pose.transform.translation.x, pose.transform.translation.y
        m.info.origin.position.x, m.info.origin.position.y = self._x_min, self._y_min
        #m.info.origin.position.x, m.info.origin.position.y = (self._x_num)/2, (self._y_num)/2
        #print(m.info.origin)
        #print((self._x_res, self._y_res))
        m.info.resolution, m.info.width, m.info.height = self._x_res, self._x_num, self._y_num
        #print(self.LogOddsToProbability(self._map))
        np.set_printoptions(threshold=sys.maxsize)
        
        arr = np.array([])
        for i, el in enumerate(np.argwhere(self.LogOddsToProbability(self._map) >.5)):
            arr = np.append(arr, [(el[0]-100)/50.0, (el[1]-100)/50.0])
        print(arr)
        print(len(self._map))
        print(len(self._map[0]))
        
        
        map_data = Float64MultiArray()
        map_data.data = arr.reshape(-1)
        
        
        self._map_pub.publish(map_data)
        
        
        
        #numpy.set_printoptions(threshold=sys.maxsize)
        print(np.unique(self.LogOddsToProbability(self._map)))
        m.data = list((self.LogOddsToProbability(self._map) * 100).astype(np.uint8).reshape(-1))
        
        
        self._vis_pub.publish(m)
