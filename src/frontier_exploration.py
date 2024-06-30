#!/usr/bin/python3
from asyncore import loop
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
import roslib
import tf
import cv2
import os
import math
from std_srvs.srv import Trigger, TriggerRequest

# action related imports
import actionlib
from FExT2.srv import posePoint
from FExT2.msg import go_to_pointAction, go_to_pointGoal

# include other files
from utils_lib.frontier_class import FrontierDetector


class FrontierExplorer:       
    def __init__(self):

        self.criterion = 0

        # Current robot pose [x, y, yaw], None if unknown            
        self.current_pose = None

        # store map as a msg to send to frontier class
        self.map_msg = None

        # action feedbacl variable
        self.dist_to_goal = math.inf

        # arbitrary flags
        self.odom_received = False
        self.map_received =  False
        self.started = True

        # map variables
        self.map_origin = None
        self.map_resolution = None

        # frontier detector class
        self.frontierDetector = FrontierDetector()
            
        # Subscribe to map: Every time new map appears, update map and its info
        self.map_subscriber = rospy.Subscriber("/projected_map", OccupancyGrid, self.projected_map_callback)

        # Subscribe to robot pose: Get robot pose
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom)

        # Publish: candidate points and frontier lines for Visualization
        self.frontier_points_pub = rospy.Publisher("/frontier_detection/vis_points",MarkerArray,queue_size=1) #occupancy grid publisher
        self.frontier_lines_pub = rospy.Publisher("/frontier_detection/vis_lines",MarkerArray,queue_size=1) #occupancy grid publisher

        # Marker Arrays used for Visualization
        self.marker_candidate_points = MarkerArray()
        self.marker_candidate_points.markers = []
        self.marker_frontier_lines = MarkerArray()
        self.marker_frontier_lines.markers = []

        # Action client to connect to move_to_point server
        self.client = actionlib.SimpleActionClient('move_to_point', go_to_pointAction)
        self.client.wait_for_server()
        self.get_goal()
        
    '''
    Action feedback computation
    '''
    def feedback_cb(self, feedback):
        self.dist_to_goal = feedback.dist_to_goal

    '''
    Odometry callback: 
    Gets current robot pose and stores it into self.current_pose
    '''
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
        self.odom_received =  True
    
    '''
    occupancy map callback: 
    Called when a new occupancy grid is received. Stores map and its info
    '''
    def projected_map_callback(self, data):
        self.map_msg = data
        self.map_received = True
        self.map_origin = [data.info.origin.position.x, data.info.origin.position.y] 
        self.map_resolution = data.info.resolution
            
    '''
    The main action server functions
    Responsible for computing the candidate point, and sending it to the move_to
    '''
    def get_goal(self):    
        
        # if we have started or if the point is reached
        while self.started or self.client.get_result(): 
            self.started = False

            # print the action feedback
            print('self.dist_to_goal',self.dist_to_goal)

            if self.odom_received and self.map_received:

                # update the map and pose in the class
                self.frontierDetector.set_mapNpose(self.map_msg, self.current_pose)

                # call the main function to get a list of candidate points according to selected IG
                candidate_pts_ordered, labelled_frontiers = self.frontierDetector.getCandidatePoint(self.criterion)
                
                # convert the occupancy candidate points to real world points
                candidate_pts_catesian = self.frontierDetector.all_map_to_position(candidate_pts_ordered)
            
                # publish the highest priority candidate point for visualization
                self.publish_frontier_points([[candidate_pts_catesian[0,0], candidate_pts_catesian[0,1]]])
                
                # publish frontier lines for visualization
                self.publish_frontier_lines(labelled_frontiers)

                # send the goal point to move_to_point server node
                self.goal = go_to_pointGoal(goal_x = candidate_pts_catesian[0,0], goal_y = candidate_pts_catesian[0,1])
                self.client.send_goal(self.goal, feedback_cb = self.feedback_cb)
                self.client.wait_for_result()
                print("Get result: ",self.client.get_result())

    #--------------    Arbitrary Functions ------------------------------------------
      
    '''
    Convert map position to world coordinates. 
    '''
    def __map_to_position__(self, p):
        mx = p[1]*self.map_resolution+self.map_origin[0] 
        my = p[0]*self.map_resolution+self.map_origin[1] 
        return [mx,my]

    '''
    Converts a list of points in map coordinates to world coordinates
    '''
    def __all_map_to_position__(self, pts):
        lst = []
        for p in pts:
            lst.append(self.__map_to_position__(p))
        return np.array(lst)
    
    #--------------    Visualization Functions

    def __create_colors__(self,combs):
        num_cols = np.ceil(np.cbrt(combs))
        r = np.arange(0, 1.01, 1/(num_cols-1))
        g = np.arange(0, 1.01, 1/(num_cols-1))
        b = np.arange(0, 1.01, 1/(num_cols-1))
        colors = []
        for red in r:
            for green in g:
                for blue in b:
                    colors.append([red,green,blue])

        colors = colors[1:]
        col_jump = int(len(colors)/combs)
        
        new_colors = []
        for i in range(0,len(colors),col_jump):
            new_colors.append(colors[i])
        new_colors.append(colors[0])

        return np.array(new_colors)

    def publish_frontier_points(self,data):   
        self.marker_candidate_points.markers = []
        for i in range(0,len(data)):
            self.myMarker = Marker()
            self.myMarker.header.frame_id = "odom"
            self.myMarker.type = self.myMarker.SPHERE # sphere
            self.myMarker.action = self.myMarker.ADD

            self.myPoint = Point()
            self.myPoint.x = data[i][0]
            self.myPoint.y = data[i][1]
            self.myMarker.pose.position = self.myPoint
            
            self.myMarker.color=ColorRGBA(0, 1, 0, 1)
            self.myMarker.scale.x = 0.2
            self.myMarker.scale.y = 0.2
            self.myMarker.scale.z = 0.2
            self.myMarker.lifetime = rospy.Duration(0)
            
            self.marker_candidate_points.markers.append(self.myMarker)
            id = 0
            for m in self.marker_candidate_points.markers:
                m.id = id
                id += 1
            self.frontier_points_pub.publish(self.marker_candidate_points)

    def publish_frontier_lines(self,labelled_map):   
        # print(np.unique(labelled_map))

        # num_fronts =  np.max(np.unique(labelled_map))

        # colors = [(0.0, 0.0, 0.33), (0.0, 0.0, 0.67), (0.0, 0.0, 1.0), (0.0, 0.33, 0.0), (0.0, 0.33, 0.33), (0.0, 0.33, 0.67), (0.0, 0.33, 1.0), (0.0, 0.67, 0.0), (0.0, 0.67, 0.33), (0.0, 0.67, 0.67), (0.0, 0.67, 1.0), (0.0, 1.0, 0.0), (0.0, 1.0, 0.33), (0.0, 1.0, 0.67), (0.0, 1.0, 1.0), (0.33, 0.0, 0.0), (0.33, 0.0, 0.33), (0.33, 0.0, 0.67), (0.33, 0.0, 1.0), (0.33, 0.33, 0.0), (0.33, 0.33, 0.33), (0.33, 0.33, 0.67), (0.33, 0.33, 1.0), (0.33, 0.67, 0.0), (0.33, 0.67, 0.33), (0.33, 0.67, 0.67), (0.33, 0.67, 1.0), (0.33, 1.0, 0.0), (0.33, 1.0, 0.33), (0.33, 1.0, 0.67), (0.33, 1.0, 1.0), (0.67, 0.0, 0.0), (0.67, 0.0, 0.33), (0.67, 0.0, 0.67), (0.67, 0.0, 1.0), (0.67, 0.33, 0.0), (0.67, 0.33, 0.33), (0.67, 0.33, 0.67), (0.67, 0.33, 1.0), (0.67, 0.67, 0.0), (0.67, 0.67, 0.33), (0.67, 0.67, 0.67), (0.67, 0.67, 1.0), (0.67, 1.0, 0.0), (0.67, 1.0, 0.33), (0.67, 1.0, 0.67), (0.67, 1.0, 1.0), (1.0, 0.0, 0.0), (1.0, 0.0, 0.33), (1.0, 0.0, 0.67), (1.0, 0.0, 1.0), (1.0, 0.33, 0.0), (1.0, 0.33, 0.33), (1.0, 0.33, 0.67), (1.0, 0.33, 1.0), (1.0, 0.67, 0.0), (1.0, 0.67, 0.33), (1.0, 0.67, 0.67), (1.0, 0.67, 1.0), (1.0, 1.0, 0.0), (1.0, 1.0, 0.33), (1.0, 1.0, 0.67), (1.0, 1.0, 1.0)]

        # colors = self.__create_colors__(num_fronts)
        # print(colors)

        self.marker_frontier_lines = MarkerArray()
        self.marker_frontier_lines.markers = []

        lines_list = []
        val_list = []
        label_list = []

        for i in range(labelled_map.shape[0]):
            for j in range(labelled_map.shape[1]):
                val = labelled_map[i,j]
                if val!=0:
                    lines_list.append([i,j])
                    # val_list.append(colors[val])
                    label_list.append(val)

        unique_numbers, counts = np.unique(label_list, return_counts=True)

        # print(unique_numbers)
        # print(counts)
        # print(lines_dict)

        for i in range(0,len(lines_list)):
            self.myMarker = Marker()
            self.myMarker.header.frame_id = "odom"
            self.myMarker.type = self.myMarker.SPHERE # sphere
            self.myMarker.action = self.myMarker.ADD
            self.myMarker.id = i

            self.myMarker.pose.orientation.x = 0.0
            self.myMarker.pose.orientation.y = 0.0
            self.myMarker.pose.orientation.z = 0.0
            self.myMarker.pose.orientation.w = 1.0

            p = self.__map_to_position__([lines_list[i][0],lines_list[i][1]])

            self.myPoint = Point()
            self.myPoint.x = p[0]
            self.myPoint.y = p[1]
            self.myMarker.pose.position = self.myPoint
            
            self.myMarker.color=ColorRGBA(0, 0, 1, 0.5)
                        # self.myMarker.color=ColorRGBA(colors[i*col_jump,0], colors[val*col_jump,1], colors[val*col_jump,2], 0.5)

            self.myMarker.scale.x = 0.1
            self.myMarker.scale.y = 0.1
            self.myMarker.scale.z = 0.05
            # self.myMarker.lifetime = rospy.Duration(0)
            
            label = label_list[i]
            # print(label)
            num_labels = counts[label-1]
            if num_labels>7:
                self.marker_frontier_lines.markers.append(self.myMarker)
        # Create DELETE markers for previously published markers
        # delete_markers = MarkerArray()
        # delete_marker = Marker()
        # delete_marker.action = Marker.DELETEALL
        # delete_markers.markers.append(delete_marker)
        # self.frontier_lines_pub.publish(delete_markers)
        
        self.frontier_lines_pub.publish(self.marker_frontier_lines)


if __name__ == '__main__':
    rospy.init_node('frontier_explorer', anonymous=True)
    n = FrontierExplorer()
    rospy.spin()
