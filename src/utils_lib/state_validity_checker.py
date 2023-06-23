import numpy as np
import math
from ompl import base as ob
from ompl import geometric as og

def wrap_angle(angle):
    """Wrap an angle to the range of -pi to pi."""
    return math.atan2(math.sin(angle), math.cos(angle))

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1, is_unknown_valid=True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        self.map_dim = None

        # map sampling resolution (size of a cell))                            
        self.resolution = None
        
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        
        # set method has been called                          
        self.there_is_map = False
        
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    

        # test position
        # self.test_pos = (-0.7, 0) # Free loc
        # # self.test_pos = (-1.0, 1.5) # Occupied loc
        # # self.test_pos = (1.0, 0.0) # Unknown loc
        # # self.test_pos = (10.0, 10.0) # Out of map loc
        # self.test_pos = (0.6, -0.4) # free but osbtacle in vicinty
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.map_dim = self.map.shape
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        # print("grid_map recieved and saved")
        # print("map_shape: ",self.map_dim)
        # print("map_origin: ",self.origin)
        # print("map_res: ",self.resolution)
        #print(self.is_valid(self.test_pos))
        # print(self.__check_entire_valid__())
    
    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose,checking_path=False): 

        # convert world robot position to map coordinates using method __position_to_map__
        # check occupancy of the vicinity of a robot position (indicated by self.distance atribude). 
        # print('pose: ', pose)
        grid_pose = self.__position_to_map__(pose)

        if self.__in_map__ == False:
            return False
        if grid_pose != []:
            # print('in map')
            grid_pose = (int(round(grid_pose[0],0)),int(round(grid_pose[1],0)))
            map_value = self.map[grid_pose]
            # print('map value: ', map_value)

            # Return True if free, False if occupied and self.is_unknown_valid if unknown.
            if map_value == 0:   # If free space, check vicinity as well
                return self.__check_vicinity__(grid_pose, self.distance,checking_path)
                # return True
            elif map_value == -1 and self.is_unknown_valid == True:
                return self.__check_vicinity__(grid_pose, self.distance,checking_path)
                # return True
            else: # if obstacle, or if its unknown and is_unknown_valid = False, return False
                return False
        else:
            # print('out of map')
            # print('pose: ', pose)
            # print('pose: ', grid_pose)
            return False

    def compute_new_goal(self,path,iter = -1):
        # print("compute_new_goal")
        step_size=0.04
        # print('path: ',path)
        # print('iter: ',iter)

        p1, p2 = [path[iter], path[iter-1]]

        dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        num_steps = dist / step_size
        num_steps= int(num_steps)
        for j in range(num_steps):
            interpolation = float(j) / num_steps  
            x = p1[0] * (1-interpolation) + p2[0] * interpolation
            y = p1[1] * (1-interpolation) + p2[1] * interpolation
            
            if self.is_valid([x,y]):
                return np.array([x,y])
    
        if len(path) > abs(iter) + 1:
            return self.compute_new_goal(path,iter-1)
        else:
            return None
        

    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):
        # TODO: convert world position to map coordinates. If position outside map return `[]` or `None`
        mx = (p[0]-self.origin[0])/self.resolution 
        my = (p[1]-self.origin[1])/self.resolution
        if self.__in_map__([mx,my]):
            return [mx,my]
        return [] 
    
    def __in_map__(self, loc):
        '''
        loc: list of index [x,y]
        returns True if location is in map and false otherwise
        '''
        [mx,my] = loc
        if mx >= self.map.shape[0]-1 or my >= self.map.shape[1]-1 or mx < 0 or my < 0:
            return False 
        return True
    
    def __check_vicinity__(self, cell, distance, checking_path=False):
        if checking_path:
            distance = distance/2
        discrete_distance = int(round(distance/self.resolution,0))
        # print('distance: ', distance)
        # print('resolution: ', self.resolution)
        # print('no of cells on either side: ', discrete_distance)
        ##############     QUESTION WHAT TO DO IF NEIGHBORHOOD IS OUT OF MAP    #######################
        for r in range(cell[0]-discrete_distance, cell[0]+discrete_distance):
            for c in range(cell[1]-discrete_distance, cell[1]+discrete_distance):
                if self.__in_map__([r,c]):
                    map_value = self.map[r,c]
                    if map_value == 100 or (map_value == -1 and self.is_unknown_valid == False): # if obstacle, or if its unknown and is_unknown_valid = False, return False
                        return False
        return True

    
    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path, step_size=0.04):
        waypoints = []  # upsampled points
        # TODO: Discretize the positions between 2 waypoints with step_size
        for i in range(len(path)-1):
            p1, p2 = path[i], path[i+1]
            # print("now checking these two points",p1,'and',p2)
            dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            # print ("the distance in between is","  ",dist)
            num_steps = dist / step_size
            num_steps= int(num_steps)
            # print ("the number of steps in between","  ",num_steps)
            for j in range(num_steps):
                interpolation = float(j) / num_steps  #the interpolation value for each step to find the pt we are checking right now
                # print ('interpolation', interpolation)
                # p = p1 * (1 - interpolation) + p2 * interpolation
                x = p1[0] * (1-interpolation) + p2[0] * interpolation
                y = p1[1] * (1-interpolation) + p2[1] * interpolation
                # print ('the point we are checking', (x, y))
                #print ('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
                waypoints.append((x,y))
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 
        # In case the robot lands on obstacle space while picking the can, allow some margin since now it is no longer obstacle space
        # print('---------------------------------')
        # waypts_to_skip = int(round(self.distance*1.2/step_size))
        # print('waypt len before: ', len(waypoints))
        # try:
        #     waypoints = waypoints[waypts_to_skip:len(waypoints)-waypts_to_skip] 
        # except:
        #     try:
        #         waypoints = waypoints[waypts_to_skip:] 
        #     except:
        #         pass
        # print('waypt len after: ', len(waypoints))
        for w in waypoints:
            # print(w)
            if self.is_valid(w,checking_path=True) == False:
                # print("------------------------")
                # print("This is the point!",w)
                # print("------------------------")
                return False
        return True

    # Given a path, returs true if the path is not in collision and false othewise.
    def CheckObstacleCollision(self,x1,x2,step_size=0.2):
        path = (x1,x2)
        waypoints = []  # upsampled points
        # TODO: Discretize the positions between 2 waypoints with step_size
        for i in range(len(path)-1):
            p1, p2 = path[i], path[i+1]
            # print("now checking these two points",p1,'and',p2)
            dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            # print ("the distance in between is","  ",dist)
            num_steps = dist / step_size
            num_steps= int(num_steps)
            # print ("the number of steps in between","  ",num_steps)
            for j in range(num_steps):
                interpolation = float(j) / num_steps  #the interpolation value for each step to find the pt we are checking right now
                #print ('interpolation', interpolation)
                #p = p1 * (1 - interpolation) + p2 * interpolation
                x = p1[0] * (1-interpolation) + p2[0] * interpolation
                y = p1[1] * (1-interpolation) + p2[1] * interpolation
                # print ('the point we are checking', (x, y))
                #print ('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
                waypoints.append((x,y))
                
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 
        for w in waypoints:
            # print(w)
            if self.is_valid(w) == False:
                return True
        return False        


                

