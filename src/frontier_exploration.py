#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf
import math
import actionlib
from FExT2.msg import go_to_pointAction, go_to_pointGoal
from utils_lib.frontier_class import FrontierDetector

class FrontierExplorer:       
    def __init__(self):
        """
        Initialize the FrontierExplorer object.
        """
        self.load_params()

        self.criterion = self.criterion
        self.robot_size = self.robot_size

        # Current robot pose [x, y, yaw], None if unknown            
        self.current_pose = None

        # Store map as a msg to send to frontier class
        self.map_msg = None

        # Action feedback variable
        self.dist_to_goal = math.inf

        # Flags
        self.odom_received = False
        self.map_received = False
        self.started = True

        # Map variables
        self.map_origin = None
        self.map_resolution = None

        # Frontier detector class
        self.frontierDetector = FrontierDetector(self.robot_size)
            
        # Subscribers
        self.map_subscriber = rospy.Subscriber(self.projected_map_topic, OccupancyGrid, self.projected_map_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.get_odom)

        # Publishers for visualization
        self.frontier_points_pub = rospy.Publisher("/frontier_detection/vis_points", MarkerArray, queue_size=1)
        self.frontier_lines_pub = rospy.Publisher("/frontier_detection/vis_lines", MarkerArray, queue_size=1)

        # Marker Arrays for Visualization
        self.marker_candidate_points = MarkerArray()
        self.marker_frontier_lines = MarkerArray()

        # Action client to connect to move_to_point server
        self.client = actionlib.SimpleActionClient('move_to_point', go_to_pointAction)
        self.client.wait_for_server()
        
        self.get_goal()

    def load_params(self):
        """
        Load parameters from the ROS parameter server.
        """
        self.criterion = rospy.get_param('/frontier_selection_criterion', 0)
        self.projected_map_topic = rospy.get_param('/topics/projected_map', '/projected_map')
        self.odom_topic = rospy.get_param('/topics/odom', '/odom')
        self.robot_size = rospy.get_param('/robot_size', 0.22)

    def feedback_cb(self, feedback):
        """
        Action feedback callback to update distance to goal.

        Parameters:
        feedback (go_to_pointFeedback): Feedback message containing the distance to goal.
        """
        self.dist_to_goal = feedback.dist_to_goal

    def get_odom(self, odom):
        """
        Odometry callback: Gets current robot pose and stores it into self.current_pose.

        Parameters:
        odom (Odometry): The odometry message.
        """
        _, _, yaw = tf.transformations.euler_from_quaternion([
            odom.pose.pose.orientation.x, 
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        ])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
        self.odom_received = True
    
    def projected_map_callback(self, data):
        """
        Occupancy map callback: Called when a new occupancy grid is received. Stores map and its info.

        Parameters:
        data (OccupancyGrid): The occupancy grid message.
        """
        self.map_msg = data
        self.map_received = True
        self.map_origin = [data.info.origin.position.x, data.info.origin.position.y]
        self.map_resolution = data.info.resolution
            
    def get_goal(self):    
        """
        The main function responsible for computing the candidate point and sending it to the move_to_point action server.
        """
        while self.started or self.client.get_result(): 
            self.started = False

            rospy.loginfo('Distance to goal: %f', self.dist_to_goal)

            if self.odom_received and self.map_received:
                self.frontierDetector.set_mapNpose(self.map_msg, self.current_pose)
                candidate_pts_ordered, labelled_frontiers = self.frontierDetector.getCandidatePoint(self.criterion)
                
                candidate_pts_catesian = self.frontierDetector.all_map_to_position(candidate_pts_ordered)
            
                self.publish_frontier_points([[candidate_pts_catesian[0, 0], candidate_pts_catesian[0, 1]]])
                self.publish_frontier_lines(labelled_frontiers)

                self.goal = go_to_pointGoal(goal_x=candidate_pts_catesian[0, 0], goal_y=candidate_pts_catesian[0, 1])
                self.client.send_goal(self.goal, feedback_cb=self.feedback_cb)
                self.client.wait_for_result()
                rospy.loginfo("Result: %s", self.client.get_result())

    def _map_to_position(self, p):
        """
        Convert map position to world coordinates.

        Parameters:
        p (list): The map position [x, y].

        Returns:
        list: The world coordinates [mx, my, 0.0].
        """
        mx = p[1] * self.map_resolution + self.map_origin[0]
        my = p[0] * self.map_resolution + self.map_origin[1]
        return [mx, my, 0.0]

    def _all_map_to_position(self, pts):
        """
        Convert a list of points in map coordinates to world coordinates.

        Parameters:
        pts (list): List of points in map coordinates.

        Returns:
        numpy.ndarray: Array of points in world coordinates.
        """
        return np.array([self._map_to_position(p) for p in pts])
    
    def _create_colors(self, combs):
        """
        Create a list of colors for visualization.

        Parameters:
        combs (int): Number of unique colors needed.

        Returns:
        numpy.ndarray: Array of RGB colors.
        """
        num_cols = np.ceil(np.cbrt(combs))
        r = np.linspace(0, 1, int(num_cols))
        g = np.linspace(0, 1, int(num_cols))
        b = np.linspace(0, 1, int(num_cols))
        colors = np.array([[red, green, blue] for red in r for green in g for blue in b])

        colors = colors[1:]
        col_jump = len(colors) // combs
        
        return np.array([colors[i] for i in range(0, len(colors), col_jump)] + [colors[0]])

    def publish_frontier_points(self, data):   
        """
        Publish candidate points for visualization.

        Parameters:
        data (list): List of candidate points [x, y].
        """
        self.marker_candidate_points.markers = []
        for i, point in enumerate(data):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = Point(point[0], point[1], 0)
            marker.color = ColorRGBA(0, 1, 0, 1)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.lifetime = rospy.Duration(0)

            marker.id = i
            self.marker_candidate_points.markers.append(marker)

        self.frontier_points_pub.publish(self.marker_candidate_points)

    def publish_frontier_lines(self, labelled_map):
        """
        Publish frontier lines for visualization.

        Parameters:
        labelled_map (numpy.ndarray): Array representing the labelled frontier map.
        """
        self.marker_frontier_lines.markers = []
        lines_list = []
        label_list = []

        for i in range(labelled_map.shape[0]):
            for j in range(labelled_map.shape[1]):
                val = labelled_map[i, j]
                if val != 0:
                    lines_list.append([i, j])
                    label_list.append(val)

        unique_numbers, counts = np.unique(label_list, return_counts=True)

        for i, point in enumerate(lines_list):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = i

            marker.pose.orientation.w = 1.0
            marker.pose.position = Point(*self._map_to_position(point))

            marker.color = ColorRGBA(0, 0, 1, 0.5)
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.05

            label = label_list[i]
            if counts[label - 1] > 7:
                self.marker_frontier_lines.markers.append(marker)

        self.frontier_lines_pub.publish(self.marker_frontier_lines)

if __name__ == '__main__':
    rospy.init_node('frontier_explorer', anonymous=True)
    FrontierExplorer()
    rospy.spin()
