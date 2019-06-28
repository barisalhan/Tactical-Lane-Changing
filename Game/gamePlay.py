# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 12:02:26 2019

@author: Baris ALHAN
"""

import gameMode
import gameDynamics
import Vehicle.VehicleControlModel
import Vehicle.VehicleDynamics

import numpy as np
import pygame 
from pygame.locals import *

class gamePlay:
    '''
        The class that actions of the game happens. We could think this
        class as the brain of the entire project.
        # TODO: gym will be connected to here.
    '''
    def __init__(self, mode, dynamics, display, veh_props, veh_model):

        self._mode = mode
        self._dynamics = dynamics
        self._display = display
        self._veh_props = veh_props
        self._veh_model = veh_model
        
        # time tick of the simulation
        self._time = 0
        
        # The velocity of the ego vehicle
        self._ego_v,
        self._ego_id,
        # The desired velocity of the ego vehicle
        self._desired_v = self.calculate_desired_v(self._dynamics.desired_min_v,
                                                            self._dynamics.desired_max_v),
        #A list that stores the velocity difference with the front vehicle for each vehicle                                   
        self._delta_v,
        
        self._window_width = (self._dynamics.gameDynamics().max_veh_inlane * 
                               (self._veh_props.width + 2 * self._veh_props.height))
        
    # The method calculates the desired velocity for the ego vehicle.
    def calculate_desired_v(self, desired_min_v, desired_max_v):
        result = np.random.uniform(desired_min_v, desired_max_v, self._mode._num_cars)
        index_ego_vehicle = int((self._mode._num_cars-1)/2)
        # ??
        result[index_ego_vehicle] = np.array([25])
        result.shape = (len(result), 1)
        result = result[0 : self._mode._num_cars]
        
        return result
        
    
    # TODO: understand the method.
    def render(self, mode = 'human'):
        return 0
   
    # PyGame related function.
    def terminate(self):
        pygame.quit()
        return False 
    
    # PyGame related function.
    def wait_for_player_to_press_key(self):
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.terminate()
                if event.type == KEYDOWN:
                    # escape quits
                    if event.key == K_ESCAPE:  
                        self.terminate()
                        return
                    elif event.key == K_LEFT:
                        return -1
                    elif event.key == K_RIGHT:
                        return 1
                    else:
                        return 0
   
    def play(self):
        #       ??
        metadata = {'render.modes':['human']}
        
        
    def reset(self):
        '''
            The variables are the same as in the constructor.
        '''
        self._time = 0
        
        self._ego_v,
        self._ego_id,
        self._desired_v = self.calculate_desired_v(self._dynamics.desired_min_v,
                                                            self._dynamics.desired_max_v),
        self._delta_v,
        
    
    '''
        One of the main functions for traffic simulation. It controls the longitudinal accelerations for each vehicle.
        For reference, please check paper itself
        
        Inputs: v_current, v_desired, d_v, d_s
            v_current : current speed 
            v_desired : desired speed
            d_v       : Speed diffrence with leading vehicle, !!! v_current - v_leading !!!
            d_s       : Gap with leading vehicle, !!! position_leading - position_current !!!
        Outputs: v_dot, x_dot, v_dot_unlimited
            v_dot           : Reference Acceleration, that value used for lane change safety check  
            x_dot           : Reference Speed    
            v_dot_unlimited : Reference Acceleration  Limited, that value used for simulation 
    '''
    def idm(self, v_current, v_desired, d_v, d_s):

        delta = 4  # Acceleration exponent
        a = 0.7  # Maximum acceleration     m/s^2 previous was 0.7
        b = 1.7  # Comfortable Deceleration m/s^2
        th = 1.6  # 2.0 #time headway=1.5 sec
        s0 = 2  # minimum gap =2.0 meters
        s_star = s0 + v_current * th + np.multiply(v_current, d_v) / (2 * np.sqrt(a * b))             #TODO: !!!!!!
        v_dot = a * (1 - np.power((np.divide(v_current, v_desired)), delta) - np.power((np.divide(s_star, d_s + 0.01)), 2))
        v_dot_unlimited = np.copy(v_dot)
        x_dot = v_current
        v_dot[v_dot < -20] = -20  # Lower bound for acceleration, -20 m/s^2
        return v_dot, x_dot, v_dot_unlimited
    
    
        
    


    # TODO: completely understand this method!
    '''
     The method generates the initial positions of vehicles.
     
     Aim: Vehicles are distributed to the highway without collisions.
     Method: Assign each vehicle to a free lane
     
     The algorithm is as follows:
       #TODO: add algorithm.      
     
     Inputs: num_veh, num_lane, init_range, delta_dist
         num_veh  : total number of vehicles
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
    def generate_init_points(self, num_veh = 9, num_lane = 3, init_range = 200, delta_dist = 25):
        
        #The result list stores the lane of each vehicle.
        lane_list = []
        #The result list stores the coordinates of each vehicle.
        #[(X_pos, Y_pos)]
        coordinates = np.zeros((num_veh, 2))
        #The result list stores the initial velocities of each vehicle.
        init_v = np.zeros((num_veh, 1))
        
        #TODO: comment here! [(LaneID : X_position)]
        coordinates_list = []
        
        #first randomly select lanes for each vehicle
        for car in range(0, num_veh):
            # Randomly chose lane id for each vehicle
            lane_list.append(np.random.randint(0, num_lane))  
        
        #The map that stores [LaneID <-> number of cars in that lane]
        fullness_of_lanes = {x: lane_list.count(x) for x in lane_list}
        #the list of the id of each lane. It is zero indexed.
        lane_ids = list(range(0, num_lane))
        
        # For each lane evaluate the exact position of each vehicle.
        # Algorithm:
        # 1. For each lane in lanes:
        # 2.     First, randomly calculate a point for
        #        the first car in that lane.
        # 3.     Afterwards, assign a position to the remanining
        #        vehicles one by one, ensuring that they do not collide.
        for lane, num_vehicles_inlane in fullness_of_lanes.items():
            # Temporary point list in the x direction
            # to store the positions of the vehicles.
            tmp_points = list([])
            # First, chose a point for the first vehicle in the selected lane
            # The second parameter in the randomness ensures that there is no colliding
            # accumulation at the end of the inital range.
            tmp_points.append(np.random.uniform(0 , init_range - ((num_vehicles_inlane - 1) * delta_dist) ))
            
            # Add this point to the list
            coordinates_list.append([lane, tmp_points[-1]])
            for car_id in range(0, num_vehicles_inlane - 1):
                # put other vehicles in that lane to the remaining space
                tmp_points.append(np.random.uniform(tmp_points[-1] + delta_dist,
                                                    (init_range - (num_vehicles_inlane - 2 - car_id) * delta_dist)))
                coordinates_list.append([lane, tmp_points[-1]])
                
        coordinates = np.asarray(coordinates_list).reshape(coordinates.shape)
        #TODO: comment here!
        coordinates = coordinates[coordinates[:, 1].argsort()]
        
        #TODO: change the coordinates to num_car
        ego_veh_id = int(np.ceil(len(coordinates) / 2) - 1)
        
        coordinates[:, 1] = coordinates[:, 1] - coordinates[ego_veh_id, 1] + self._window_width / 20
        # initial velocities for the ego vehicle => 10m/s 15 m/s
        # TODO: parametrize here!
        init_v[ego_veh_id] = np.random.uniform(10, 15)


        for rear_id in range(0, ego_veh_id):
             # 26.4, 33.3  , randomly define initial speeds for rear vehicles  
            init_v[rear_id] = np.random.uniform(15, 25) 
        for front_id in range(ego_veh_id + 1, num_veh):
             # 16.7, 23.6 , randomly define initial speeds for front vehicles
            init_v[front_id] = np.random.uniform(10, 12) 


        return coordinates, ego_veh_id, init_v, lane_ids        
    
    
    # TODO: ask what's going on here.
    #Calucate delta_v and delta_dist.
    def generate_deltas(self, current_states, num_cars, lane_ids, current_v):
        
        delta_v = np.zeros((num_cars, 1))
        delta_dist = np.zeros((num_cars, 1))
        
        for lane_id in lane_ids:
            
            idx = np.where(current_states[:, 0] == lane_id)
            sorted_idx = np.argsort(current_states[idx, 1])
            idx_new = idx[0][sorted_idx]

            dummy_v = 0        # If there id no vehicle in front d_v equals to 0
            dummy_s = 10 ** 5  # If there id no vehicle in front d_s equals to 100000 m
            for x, val in reversed(list(enumerate(idx_new[0]))):
                if dummy_v == 0:
                    dummy_v = current_v[val]
                delta_v[val] = current_v[val] - dummy_v
                delta_dist[val] = dummy_s - current_states[val, 1] - 0
                dummy_v = current_v[val]
                dummy_s = current_states[val, 1]
        return delta_v, delta_dist
    
    
    