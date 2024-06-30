#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import actionlib
from math import sqrt

from utils_lib.state_validity_checker import StateValidityChecker
from utils_lib.path_planners import compute_path
from utils_lib.controllers import move_to_point, move_to_point_smooth

from FExT2.msg import go_to_pointAction, go_to_pointFeedback, go_to_pointResult 

class OnlinePlanner:

    def __init__(self):
        """
        Initialize the OnlinePlanner object.
        """
        self.load_params()

        # List of points which define the plan. None if there is no plan
        self.path = []

        # State Validity Checker object                                                 
        self.svc = StateValidityChecker(self.distance_threshold)

        # Current robot pose [x, y, yaw]        
        self.current_pose = None

        # Goal where the robot has to move, None if it is not set                                                                   
        self.goal = None

        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()

        # Status
        self.is_moving = False
        self.reached = False         

        # PUBLISHERS
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)  # Publisher for sending velocity commands
        self.marker_pub = rospy.Publisher('/path_marker', Marker, queue_size=1)  # Publisher for visualizing the path
        
        # SUBSCRIBERS
        self.gridmap_sub = rospy.Subscriber("/projected_map", OccupancyGrid, self.get_gridmap)  # subscriber to gridmap_topic
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom)  # subscriber to odom_topic

        # Action server
        self._as = actionlib.SimpleActionServer('move_to_point', go_to_pointAction, execute_cb=self.execute_action, auto_start=False)
        self._as.start()

        # Timer for the controller
        rospy.Timer(rospy.Duration(0.05), self.controller)

        # Initialize feedback and result messages
        self._feedback = go_to_pointFeedback()
        self._result = go_to_pointResult()

    def load_params(self):
        """
        Load parameters from the ROS parameter server.
        """
        self.planner_config = rospy.get_param('~planner_config', 'BIT-')
        self.curved_controller = rospy.get_param('~curved_controller', False)

        self.Kv = rospy.get_param('/controller/Kv', 0.5)
        self.Kw = rospy.get_param('/controller/Kw', 0.5)
        self.v_max = rospy.get_param('/controller/v_max', 0.15)
        self.w_max = rospy.get_param('/controller/w_max', 0.3)
        self.Kp = rospy.get_param('/controller/Kp', 10)
        self.Ki = rospy.get_param('/controller/Ki', 10)
        self.Kd = rospy.get_param('/controller/Kd', 10)
        self.dt = rospy.get_param('/controller/dt', 0.05)

        self.dominion = np.array(rospy.get_param('/dominion', [-15.0, 15.0]))
        self.distance_threshold = rospy.get_param('/robot_size', 0.22)
        self.cmd_vel_topic = rospy.get_param('/cmd_vel_topic', '/cmd_vel')

    def execute_action(self, goal):
        """
        Execute the action to move the robot to a specified point.

        Parameters:
        goal (go_to_pointGoal): The goal containing target coordinates.
        """
        rospy.loginfo('Goal received: x=%f, y=%f', goal.goal_x, goal.goal_y)
        
        self.goal = np.array([goal.goal_x, goal.goal_y])
        self.is_moving = True
        self.path = self.plan()

        self._feedback.dist_to_goal = 0.0
        if self.reached:
            rospy.loginfo('Reached the goal')
            self._result.success = True
            self._as.set_succeeded(self._result)
        
        while not self.reached:
            self._feedback.dist_to_goal = self.distance_to_goal()
            self._as.publish_feedback(self._feedback)
            if self.reached:
                rospy.loginfo('Success: Reached the goal')
                self._result.success = True
                self._as.set_succeeded(self._result)
                self.reached = False
                break
            if self._as.is_preempt_requested():
                rospy.loginfo('Move to point action preempted')
                self._as.set_preempted()
                break

    def get_odom(self, odom):
        """
        Odometry callback: Gets current robot pose and stores it into self.current_pose.

        Parameters:
        odom (Odometry): The odometry message.
        """
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw]) 

    def get_gridmap(self, gridmap):
        """
        Occupancy map callback: Called when a new occupancy grid is received. Stores map and its info,
        checks if the path is valid, and recomputes the path if necessary.

        Parameters:
        gridmap (OccupancyGrid): The occupancy grid message.
        """
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 0.1:            
            self.last_map_time = gridmap.header.stamp

            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)

            if self.path is not None and len(self.path) > 0:
                total_path = [self.current_pose[0:2]] + self.path

                if not self.svc.is_valid(self.goal):
                    self.goal = self.svc.compute_new_goal(total_path)
                    if self.goal is None:
                        self.is_moving = False
                        self.reached = True
                    else:
                        self.path = self.plan()
                elif not self.svc.check_path(total_path):
                    rospy.logwarn("Invalid Path")
                    self.path = self.plan()

    def plan(self):
        """
        Create a plan from the current position to self.goal.

        Returns:
        list: List of waypoints [x, y] that compose the plan.
        """
        if self.current_pose is None or self.goal is None or self.svc.origin is None or self.svc.resolution is None:
            rospy.logwarn("Planning is not possible. Waiting for necessary data.")
            return []

        path = []
        trial = 0

        while len(path) <= 1 and trial < 5:
            rospy.loginfo("Compute new path")
            path = compute_path(self.planner_config, self.current_pose, self.goal, self.svc, self.dominion, max_time=3.0) 
            trial += 1
            
        if path:
            rospy.loginfo("Path found")
            self.publish_path(path)
            del path[0]                 
        
        return path

    def controller(self, event):
        """
        Compute velocity commands to reach the next waypoint in the path.
        Sends zero velocity commands if there is no active path.

        Parameters:
        event (TimerEvent): The event that triggered the callback.
        """
        v, w = 0, 0

        if self.path and len(self.path) > 0:
            if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < 0.1:
                del self.path[0]
                if not self.path:
                    rospy.loginfo("Final position reached!")
                    self.goal = None
                    self.is_moving = False
                    self.reached = True
            else:
                if self.curved_controller:
                    v, w = move_to_point_smooth(self.current_pose, self.path[0], Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, dt=self.dt)
                else:
                    v, w = move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)
        
        self._send_command(v, w)

    def distance_to_goal(self):       
        """
        Calculate the distance between the current pose and the goal pose.

        Returns:
        float: The distance to the goal.
        """
        if self.goal is None:
            return 0.0
        return sqrt((self.goal[0] - self.current_pose[0])**2 + (self.goal[1] - self.current_pose[1])**2)

    def _send_command(self, v, w):
        """
        Transform linear and angular velocity (v, w) into a Twist message and publish it.

        Parameters:
        v (float): Linear velocity.
        w (float): Angular velocity.
        """
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)

    def publish_path(self, path):
        """
        Publish a path as a series of line markers.

        Parameters:
        path (list): List of waypoints [x, y] that compose the plan.
        """
        if len(path) > 1:
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.ns = 'path'
            marker.action = Marker.DELETE
            marker.lifetime = rospy.Duration(0)
            self.marker_pub.publish(marker)

            marker.action = Marker.ADD
            marker.scale.x = 0.03
            marker.pose.orientation.w = 1

            color_red = ColorRGBA(1, 0, 0, 1)
            color_blue = ColorRGBA(0, 0, 1, 1)

            marker.points.append(Point(self.current_pose[0], self.current_pose[1], 0.0))
            marker.colors.append(color_blue)
            
            for point in path:
                marker.points.append(Point(point[0], point[1], 0.0))
                marker.colors.append(color_red)
            
            self.marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('online_planner')
    OnlinePlanner()
    rospy.spin()
