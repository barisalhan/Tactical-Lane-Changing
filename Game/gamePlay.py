# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 12:02:26 2019

@author: Baris ALHAN
"""

import gameMode

import numpy as np

class gamePlay:
    
    def __init(self, desired_min, desired_max):
        
        self._mode = gameMode.gameMode()
        
        self._time = 0
       
        self._V,
        self._desired_V = self.calculate_desired_velocities(desired_min, desired_max),
        self._delta_V,
       
   
    def play(self):
        #       ??
        metadata = {'render.modes':['human']}
        
        
    def reset(self):
        self._time = 0
        
        self._V,
        self._desired_V = self.calculate_desired_velocities(desired_min, desired_max),
        self._delta_V,
        
        
    def calculate_desired_velocities(self, desired_min, desired_max):
        result = np.random.uniform(desired_min, desired_max, self._mode._num_cars)
        index_ego_vehicle = int((self._mode._num_cars-1)/2)
        # ??
        result[index_ego_vehicle] = np.array([25])
        result.shape = (len(result), 1)
        result = result[0 : self._mode._num_cars]
        
        return result



    '''
     The method generates the initial positions of vehicles.
     
     Aim: Vehicles are distributed to the highway without collisions.
     Method: Assign each vehicle to a free lane
     
     The algorithm is as follows:
       #TODO: add algorithm.      
     
     Inputs: num_car, num_lane, init_range, delta_dist
         num_car   : total number of vehicles
         num_lane  : total number of lanes
         init_range : range of the initialization horizon(meters)
         delta_dist: minimum distance between vehicles (meters)
         
     Outputs: coordinates, ego_veh_id, init_v, lane_ids
         coordinates : Position of each vehicle
         ego_veh_id : ego vehicle ID    
         init_v      : Initial speed for each vehicle
         lane_ids    : Lane IDs (0 indexed) 
    '''
    #TODO: Check the safety of delta_dist
    def generate_init_points(self, num_car = 9, num_lane = 3, init_range = 200, delta_dist = 25):
        
        #The result list stores the lane of each vehicle.
        lane_list = []
        #TODO: comment here!
        coordinates_list = []
        #The result list stores the coordinates of each vehicle.
        #[(X_pos, Y_pos)]
        coordinates = np.zeros((num_car, 2))
        #The result list stores the initial velocities of each vehicle.
        init_v = np.zeros((num_car, 1))
        
        #first randomly select lanes
        for car in range(0, num_car):
            # Randomly chose lane id for each vehicle
            lane_list.append(np.random.randint(0, num_lane))  
        
        #The map that stores [LaneID: number of cars in that lane]
        fullness_of_lanes = {x: lane_list.count(x) for x in lane_list}
        #the list of the id of each lane. It is zero indexed.
        lane_ids = list(range(0, num_lane))
        
        #TODO:you are right here!
        # for each lane evaluate the exact position of each vehicle.
        for key, value in fullness_of_lanes.items():
            point = list([])
            # chose a point for first vehicle in selected lane
            point.append(np.random.uniform(0, d_long - (value - 1) * delta_d)) 
            coordinates_list.append([key, point[-1]])
            for car_id in range(0, value - 1):
                # put other vehicles to remaining space
                point.append(np.random.uniform(point[-1] + delta_d, (d_long - (value - 2 - car_id) * delta_d)))
                coordinates_list.append([key, point[-1]])
                
                
        coordinates = np.asarray(coordinates_list).reshape(coordinates.shape)
        coordinates = coordinates[coordinates[:, 1].argsort()]
        ego_veh_id = int(np.ceil(len(coordinates) / 2) - 1)
        coordinates[:, 1] = coordinates[:, 1] - coordinates[ego_veh_idx, 1] + WindowWidth / 20
        # initial velocities for the ego vehicle => 10m/s 15 m/s
        v_init[ego_veh_id] = np.random.uniform(10, 15)


        for rear_id in range(0, ego_veh_id):
             # 26.4, 33.3  , randomly define initial speeds for rear vehicles  
            init_v[rear_id] = np.random.uniform(15, 25) 
        for front_id in range(ego_veh_idx + 1, ncar):
             # 16.7, 23.6 , randomly define initial speeds for front vehicles
            init_v[front_id] = np.random.uniform(10, 12) 


        return coordinates, ego_veh_id, init_v, lane_ids        
        
        
        
    