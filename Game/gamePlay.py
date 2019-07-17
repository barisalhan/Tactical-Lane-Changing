# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 12:02:26 2019

@author: Baris ALHAN
"""
import Vehicle.VehicleControlModel
import Vehicle.VehicleDynamics
from Vehicle.VehicleControl.vehicleAIController import vehicleAIController
from Display.display import display


from Vehicle.VehicleDynamics.vehiclePhysicalProperties import vehiclePhysicalProperties
from Vehicle.VehicleControlModel.dynModel import dynModel

from Game.gameDynamics import gameDynamics
from Game.gameMode import gameMode

import numpy as np
import pygame 
from pygame.locals import *

class gamePlay:
    '''
        The class that actions of the game happens. We could think this
        class as the brain of the entire project.
        
        All the velocity calculations are made in the unit of m/s.
    '''
    # TODO: add reset method.
    def __init__(self):

        # time tick of the simulation (s)
        self._time = 0
        # Analog of the real time required to do just one substep.
        self._dt = 0.05
        
        # The below constructors are created with default parameters,
        # to read about the parameters of a class, go to the related class.
        #######################################################################
        #####                       INITIALIZATION                        #####
        #######################################################################
        self._mode = gameMode()
        self._dynamics = gameDynamics()
        self._veh_props = vehiclePhysicalProperties()
        self._veh_model = dynModel()
        self._AIController = vehicleAIController(self._dt)
        self._display = display(self)
        #######################################################################
        #######################################################################
        
        
        #######################################################################
        #####                           VEHICLES                          #####
        #######################################################################
        # The id of the ego vehicle.
        self._ego_id = int((self._dynamics._num_veh-1)/2)
        
        # Coordinates of the each vehicle [LaneID : X_pos]
        self._veh_coordinates = self.generate_init_points()
        self._init_x_point_of_ego = self._veh_coordinates[self._ego_id, 1]
        # Velocities of the each vehicle (m/s)
        self._velocities = self.generate_init_velocities()
        
        # The list that stores the desired max. velocities for each vehicle.
        self._desired_v = self.calculate_desired_v(self._dynamics._desired_min_v,
                                                            self._dynamics._desired_max_v)
        
        # The lists that stores the velocity and distance differences with the front vehicle for each vehicle
        self._delta_v, self._delta_dist = self.generate_deltas(self._veh_coordinates, self._velocities)
        #######################################################################
        #######################################################################
        
    ###########################################################################
        
        
    
    '''
     The method generates the initial positions of vehicles.
     
     Aim: Vehicles are distributed to the highway without collisions.
     
     The algorithm is as follows:
         1. Assign each vehicle to a free lane
             (For each lane evaluate the exact position of each vehicle.)
             2. For each lane in lanes:
                 3. First, randomly calculate a point
                    for the first car in that lane.
                 4. Afterwards, assign a position to the remanining
                    vehicles one by one, ensuring that they do not collide.
     
    The coordinates list is sorted at the end and by that way the ego vehicle 
    is always the vehicle in the middle.
        
     Inputs: init_range, delta_dist
         init_range : range of the initialization horizon(meters)
         delta_dist: minimum distance between vehicles (meters)
         
     Outputs: coordinates
         coordinates : Position of vehicles => [LaneID, X_pos]
    '''
    def generate_init_points(self, init_range = 200, delta_dist = 25):
        
        # Safety check for the distance of the vehicles.
        if delta_dist<10 :
            delta_dist = 10
            
        #The result list stores the lane of each vehicle.
        lane_list = []
        #The result list stores the coordinates of each vehicle.
        #[LaneID, X_pos)]
        coordinates = np.zeros((self._dynamics._num_veh, 2))
        
               
        #first randomly select lanes for each vehicle
        for veh in range(0, self._dynamics._num_veh):
            # Randomly chose lane id for each vehicle
            lane_list.append(np.random.randint(0, self._dynamics._num_lane))  
        
        #The map that stores [LaneID <-> number of vehicles in that lane]
        fullness_of_lanes = {x: lane_list.count(x) for x in lane_list}        
        
        # Temporary list to store the coordinates of the each vehicle.
        # [(LaneID : X_position)]
        tmp_coordinates = []
        
        # 2nd step of the algorithm.
        for lane, num_vehicles_inlane in fullness_of_lanes.items():
            # First, chose a point for the first vehicle in the selected lane
            # The second parameter in the randomness ensures that there is no
            # accumulation of vehicles at the end of the inital range.
            tmp_point = (np.random.uniform(0 , init_range - ((num_vehicles_inlane - 1) * delta_dist) ))
            
            tmp_coordinates.append([lane, tmp_point])
            
            for veh_id in range(0, num_vehicles_inlane - 1):
                # put other vehicles in that lane to the remaining space
                tmp_point = (np.random.uniform(tmp_point + delta_dist,
                                                    (init_range - (num_vehicles_inlane - 2 - veh_id) * delta_dist)))
                tmp_coordinates.append([lane, tmp_point])
                
        coordinates = np.asarray(tmp_coordinates).reshape(coordinates.shape)
        coordinates = coordinates[coordinates[:, 1].argsort()]
        coordinates[:, 1] = coordinates[:, 1] - coordinates[self._ego_id, 1] + self._display._window_width / 20
    
        return coordinates          
    
    # The method that returns the initial velocities of vehicles
    def generate_init_velocities(self):
        
        #The result list stores the initial velocities of each vehicle.
        init_v = np.zeros((self._dynamics._num_veh, 1))
        
        # initial velocity for the ego vehicle is between 10m/s and 15 m/s
        init_v[self._ego_id] = np.random.uniform(10, 15)
        
        # randomly define initial speeds for the rear vehicles  
        for rear_id in range(0, self._ego_id):
            init_v[rear_id] = np.random.uniform(26.4, 33.3)
            
        # randomly define initial speeds for the front vehicles   
        for front_id in range(self._ego_id + 1, self._dynamics._num_veh):   
            init_v[front_id] = np.random.uniform(16.7, 23.6) 
        
        return init_v
    
    # TODO: it is not readable nor understandable.
    # Calculates delta_v and delta_dist with the front vehicle for each vehicle
    def generate_deltas(self, coordinates, velocities):
        
        num_veh = self._dynamics._num_veh
        num_lane = self._dynamics._num_lane
        
        delta_v = np.zeros((num_veh, 1))
        delta_dist = np.zeros((num_veh, 1))
        
        for lane_id in range(num_lane):
            # Detect the vehicles in that lane_id.
            idx = np.where(coordinates[:, 0] == lane_id)
            # sort them by their x position
            sorted_idx = np.argsort(coordinates[idx, 1])
            idx_new = idx[0][sorted_idx]

            # If there is no vehicle in front delta_v equals to 0
            dummy_v = 0
            # If there is no vehicle in front delta_dist equals to 100000 m
            dummy_s = 10 ** 5
            
            for x, val in reversed(list(enumerate(idx_new[0]))):
                if dummy_v == 0:
                    dummy_v = velocities[val]
                delta_v[val] = velocities[val] - dummy_v
                delta_dist[val] = dummy_s - coordinates[val, 1] - 0
                dummy_v = velocities[val]
                dummy_s = coordinates[val, 1]
                
        return delta_v, delta_dist
    
    
    # The method calculates the desired max. velocities for the each vehicle.
    # The desired max. velocity of the ego car is 25 m/s
    def calculate_desired_v(self, desired_min_v, desired_max_v):
        
        result = np.random.uniform(desired_min_v, desired_max_v, self._dynamics._num_veh)
        result[self._ego_id] = np.array([25])
        result.shape = (len(result), 1)
        result = result[0 : self._dynamics._num_veh]
        
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
                if event.type == pygame.QUIT:
                    self.terminate()
                if event.type == pygame.KEYDOWN:
                    # escape quits
                    if event.key == pygame.K_ESCAPE:  
                        self.terminate()
                        return
                    elif event.key == pygame.K_LEFT:
                        return -1
                    elif event.key == pygame.K_RIGHT:
                        return 1
                    else:
                        return 0
    
    
    # TODO: change the place of this method and explain what it does.
    # RL related function.
    # check paper table 2.
    def get_input_states(self, states, V_vehicles, t):
        # s_max = max(abs(states[:, 1] - states[self.ego_veh_id, 1]))
        # s_max = v_ego_max * t
        # v_max = max(V_vehicles)
        s_max = self._goal_distance # distance_long/2+ safety margin will be 50
        v_max = self._desired_v[self._ego_id]
        input_vector = np.zeros((3*self._dynamics._num_veh, 1))
        input_vector[0] = np.random.normal(V_vehicles[self._ego_id] / v_max, 0.1) # our speed unceert
    
        if states[self._ego_id, 0] == 0:
            input_vector[1] = 0
        else:
            input_vector[1] = 1
        if states[self._ego_id, 0] == (self._dynamics._num_lane-1):
            input_vector[2] = 0
        else:
            input_vector[2] = 1
    
        for idx in range(0, self._ego_id):
            input_vector[3 * (idx + 1)] = np.random.normal(((states[idx, 1] - states[self._ego_id, 1]) / s_max), 0.3) # uncert for distance
            input_vector[3 * (idx + 1) + 1] = np.random.normal(V_vehicles[idx] / v_max, 0.2) # uncert for speed
            input_vector[3 * (idx + 1) + 2] = (states[idx, 0] - states[self._ego_id, 0])/2
    
        for idx in range(self._ego_id+1, self._dynamics._num_veh):
            input_vector[3 * (idx )] = np.random.normal((states[idx, 1] - states[self._ego_id, 1]) / s_max, 0.3) # uncert for distance
            input_vector[3 * (idx) + 1] = np.random.normal(V_vehicles[idx] / v_max, 0.2) # uncert for speed
            input_vector[3 * (idx) + 2] = (states[idx, 0] - states[self._ego_id, 0])/2
        return input_vector
    
    
    # TODO: explain the general algorithm.
    '''
        1. Determine the longitudinal movement of each vehicle.
            a. Calculate the acceleration in the x direction by using IDM.
        2. Determine the lane change movement.
            a. Find the surrounding vehicles.
            b. Take the lane change decisions according to MOBIL.
    '''
    def step(self, action):
        
        # holds whether the RL episode done or not.
        is_done = False
        # If ego vehicle get too close with any other vehicle 
        # the near_collision becomes true
        near_collision = 0
        # If ego vehicle collides with any other vehicle
        # hard_collision becomes true.
        hard_collision = 0
        # The reward of the RL
        reward = 0.0
        # The list that holds the lane change decisions of the vehicles.
        # Decisions of the ego vehicle are determined by RL if it is enabled,
        # else it is directly taken from the user input.
        # Decisions of the other vehicles are determined by the movement model.
        decisions = np.zeros((self._dynamics._num_veh,1))
        # We can think gains list as the priority level of each vehicle 
        # to change the lane. During the execution, the vehicle with top priority
        # will change the lane first.
        gains = np.zeros((self._dynamics._num_veh,1))
        # Holds the target lane after lane change decision is made for each vehicle.
        target_lanes = self._veh_coordinates[:, 0]
        # To detect collisions, the estimated time of collision is calculated.
        # The low and high range are used to determine the difference between
        # the near and hard collision. 
        TIME_OF_COLLISION_LOW = 0.1
        TIME_OF_COLLISION_HIGH = 1.8
        
        #######################################################################
        #####                  AIController Decisions                    ######
        #######################################################################
        accelerations = self._AIController.IDM(self._velocities, self._desired_v,
                                              self._delta_v, self._delta_dist)
        
        # if the traffic rule enables lane changing
        if self._mode._rule_mode == 1 or self._mode._rule_mode == 2:
           if self._time > 0:
               for veh in range(0, self._dynamics._num_veh):
                   # Finding the surrounding vehicles.
                   ll_id, lf_id, ml_id, mf_id, rl_id, rf_id = self._AIController.get_surrounding_vehs(self._veh_coordinates, veh)
                   
                   decisions[veh], gains[veh] = self._AIController.MOBIL(veh, self._veh_coordinates[veh],
                                        accelerations[veh], self._velocities[veh], self._desired_v[veh], 
                                                ll_id, lf_id, ml_id, mf_id, rl_id, rf_id)
        #######################################################################
        #######################################################################
        
        
        #######################################################################
        ######          Evaluating the decision of the Ego Vehicle       ######
        #######################################################################
        if action != 0.0:
            # There is a punishment to reward the less-step-required action.
            reward += -0.1
            
            target_lanes[self._ego_id] = self._veh_coordinates[self._ego_id, 0] + action
            
            # If ego vehicle decided to change the lane
            iterate = True
            
            while iterate:
                # Safety check
                if target_lanes[self._ego_id] not in range(0, self._dynamics._num_lane):
                    target_lanes[self._ego_id] = self._veh_coordinates[self._ego_id, 0]
                    action = 0
                    reward += -1
                    iterate = False
                
                # Updating the x position of the vehicles
                self._veh_coordinates[:, 1:] = \
                    self._veh_coordinates[:, 1:] + (self._velocities * self._dt)
                # Updating the velocities of the vehicles
                self._velocities = self._velocities + (accelerations * self._dt)
                # Updating the time of the simulation
                self._time = self._time + self._dt
                
                ### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!******
                ### INCLUDES ALL VEHICLES
                x_pos = self._veh_coordinates[:, 1:]
                y_pos = self._veh_coordinates[:, 0:1 ]
                psi = np.zeros((self._dynamics._num_veh))
                
                
                [_, y_pos , psi, _] = \
                    self._AIController.lane_change(x_pos, y_pos, psi,
                                self._velocities, target_lanes)
                
                for id in range(0, self._dynamics._num_veh):
                    self._veh_coordinates[id, 0] = self._AIController.round_with_offset( \
                                         y_pos[id], decisions[id])
                    
                self._delta_v, self._delta_dist = self.generate_deltas(self._veh_coordinates, self._velocities)
                ### INCLUDES ALL VEHICLES END**********************************
                
                time_of_collision = np.divide(self._delta_dist, self._delta_v + 0.01)
                
                if len(time_of_collision[(time_of_collision >= TIME_OF_COLLISION_LOW) & (
                        time_of_collision < TIME_OF_COLLISION_HIGH)]) > 0:
                    near_collision = 1
                
                # TODO: Ask is it okey to detect hard collision by this method
                if len(time_of_collision[(time_of_collision < TIME_OF_COLLISION_LOW)]) > 0:
                    hard_collision = 1
                    is_done = True
                    iterate = False
                
                observation = self.get_input_states(self._veh_coordinates[:, :], self._velocities[:], self._time)         
                
                self._display.env_update()
                self._display._main_clock.tick()
                pygame.event.pump()
                
                if iterate:
                    if abs(target_lanes[self._ego_id] - self._veh_coordinates[self.ego_veh_id, 0]) < 0.1:
                        action = 0
                        iterate = False
                
                if np.sum(abs(accelerations)) < 0.05:
                    is_done = True
            
            self.distance_traveled = self._veh_coordinates[self._ego_id, 1] - self._init_x_point_of_ego
            
            if self.distance_traveled >= self._GOAL_DISTANCE:
                reward += 10
                is_done = True
            if hard_collision:
                reward = -10
            else:
                reward += (-near_collision) * 5
                reward += 100*(self._velocities[self._ego_id] - 10) / self._desired_v[self._ego_id]
                return observation, reward, is_done, {}
            
            
            
        #######################################################################
        #######################################################################        
        
       
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    