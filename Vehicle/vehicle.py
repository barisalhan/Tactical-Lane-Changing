# -*- coding: utf-8 -*-
"""
Created on Wed Jul 24 17:17:45 2019

@author: Baris ALHAN
"""
import math,pdb
import numpy as np

from Vehicle.vehicleAIController import vehicleAIController as AIController
from Vehicle.vehicleControlModels import PID
from Vehicle.vehicleControlModels import dynModel as DynModel


class vehicle():
 
    def __init__(self,
                 game,
                 vehcl_id,
                 is_ego):
        self._game = game
        #: int: Each vehicle has its own unique id.
        self._id = vehcl_id

        #: pair of floats: (Lane_id, X_pos)
        #self._position = position
        
        #: bool: If the vehicle is controlled by user, it is called ego.
        self._is_ego = is_ego
        #: AIController(class): Non-ego agents are directed by AIController.
        self._AIController = None
        #: bool: Is the vehicle currently in the process of lane changing?
        self._is_lane_changing = False
        #  0 => Go straight
        #  1 => Go to the right
        # -1 => Go to the left
        #: int: 
        self._lane_change_decision = 0
        #: int: Id of the target lane -> [0, num_of_lanes)
        self._target_lane = -1
        #: float: Heading angle of the vehicle. It is used while changing lane.
        self._psi = 0
        
        if self._is_ego==False :
            self._AIController = AIController(self)
      
        
    # Two-Point Visual Control Model of Steering
    # For reference, please check the paper itself.
    # Perform lane change in continuous space with using controller
    # psi -> heading angle.
    def lane_change(self, pos, psi, v, target_lane):
        
        pid = PID()
        dynModel = DynModel()

        x_pos, y_pos = pos[1], pos[0]
        
        dify = target_lane - y_pos
        # Two points are seleceted within 5 meters and 100 meters,
        # then angles are calculated and fed to PID
        near_error = np.subtract(np.arctan2(dify, 5), psi)
        far_error = np.subtract(np.arctan2(dify, 100), psi)
        pid_out = pid.update(near_error, far_error, self._game._dt)

        z = [x_pos, y_pos, psi, v, self._game._dt]

        x_next, y_next, psi_next = dynModel.update(z, pid_out)
        
        if (self.check_lane_change_done(y_pos)):
            y_next = target_lane
            
        return x_next, y_next, psi_next
    
    def check_lane_change_done(self, y_pos):
        if abs(self._target_lane - y_pos) < 0.05:
            self._is_lane_changing = False
            self._lane_change_decision = 0
            print("{} completed the lane change",format(self._id))
            
    ###########################################################################
    ######                    STATIC METHODS                              #####
    ###########################################################################
    
    '''
     This static method that generates the initial positions of vehicles.
     
     Aim: Vehicles are distributed to the highway without collisions.
     
     The algorithm is as follows:
         1. Assign each vehicle to a free lane
             (2. Afterwards, For each lane, evaluate the exact position of each vehicle.)
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
    
    @staticmethod
    def generate_init_positions(ego_id,
                                 num_vehcl,
                                 num_lane,
                                 window_width,
                                 init_range=100,
                                 delta_dist=12):
        
        # Safety check for the distance of the vehicles.
        if delta_dist < 10:
            delta_dist = 10
    
        #The result list stores the lane of each vehicle.
        lane_list = []
        #The result list stores the positions of each vehicle.
        #[(LaneID, X_pos)]
        positions = np.zeros((num_vehcl, 2))
        
        #first randomly select lanes for each vehicle
        for veh in range(0, num_vehcl):
            # Randomly chose lane id for each vehicle
            lane_list.append(np.random.randint(0, num_lane))
    
        #The map that stores [LaneID <-> number of vehicles in that lane]
        fullness_of_lanes = {x: lane_list.count(x) for x in lane_list}
    
        # Temporary list to store the positions of the each vehicle.
        # [(LaneID : X_pos)]
        tmp_positions = []
    
        # 2nd step of the algorithm.
        for lane, num_vehicles_inlane in fullness_of_lanes.items():
            # First, chose a point for the first vehicle in the selected lane
            # The second parameter in the randomness ensures that there is no
            # accumulation of vehicles at the end of the inital range.
            tmp_point = (np.random.uniform(
                0, init_range - ((num_vehicles_inlane - 1) * delta_dist)))
    
            tmp_positions.append([lane, tmp_point])
    
            for veh_num in range(0, num_vehicles_inlane - 1):
                # put other vehicles in that lane to the remaining space
                tmp_point = (np.random.uniform(
                    tmp_point + delta_dist, (init_range -
                                             (num_vehicles_inlane - 2 - veh_num) * delta_dist)))
                
                tmp_positions.append([lane, tmp_point])
        
        positions = np.asarray(tmp_positions).reshape(positions.shape)
        positions = positions[positions[:, 1].argsort()]
        positions[:, 1] = positions[:, 1] - positions[ego_id,
                                                            1] + window_width / 20
    
        return positions

    # The method that returns the initial velocities of vehicles
    @staticmethod
    def generate_init_velocities(ego_id,num_vehcl):
        #The result list stores the initial velocities of each vehicle.
        init_v = np.zeros((num_vehcl))
        # initial velocity for the ego vehicle is between 10m/s and 15 m/s
        init_v[ego_id] = np.random.uniform(10, 15)

        # randomly define initial speeds for the rear vehicles
        for rear_id in range(0, ego_id):
            init_v[rear_id] = np.random.uniform(26.4, 33.3)
        # randomly define initial speeds for the front vehicles
        for front_id in range(ego_id + 1, num_vehcl):
            init_v[front_id] = np.random.uniform(16.7, 23.6)

        return init_v
    
    # The method calculates the desired max. velocities for the each vehicle.
    # The desired max. velocity of the ego vehicle is 25 m/s
    @staticmethod
    def calculate_desired_v(ego_id,
                            num_vehcl,
                            desired_min_v,
                            desired_max_v):

        result = np.random.uniform(desired_min_v,
                                   desired_max_v,
                                   num_vehcl)
        result[ego_id] = np.array([25])
        result.shape = (len(result))

        return result
    
    # TODO: it is not readable nor understandable and I cannot change this method.
    # Calculates delta_v and delta_dist with the front vehicle for each vehicle
    @staticmethod
    def calculate_deltas(coordinates, velocities, num_vehcl, num_lane):

        delta_v = np.zeros((num_vehcl))
        delta_dist = np.zeros((num_vehcl))

        for lane_id in range(num_lane):
            # Detect the vehicles in that lane_id.
            idx = np.where(coordinates[:, 0] == lane_id)
            # sort them by their x position
            sorted_idx = np.argsort(coordinates[idx, 1])
            idx_new = idx[0][sorted_idx]

            # If there is no vehicle in front delta_v equals to 0
            dummy_v = 0
            # If there is no vehicle in front delta_dist equals to 100000 m
            dummy_s = 10**5

            for x, val in reversed(list(enumerate(idx_new[0]))):
                if dummy_v == 0:
                    dummy_v = velocities[val]
                delta_v[val] = velocities[val] - dummy_v
                delta_dist[val] = dummy_s - coordinates[val, 1] - 0
                dummy_v = velocities[val]
                dummy_s = coordinates[val, 1]

        return delta_v, delta_dist
    
    # DEPRECATED!
    @staticmethod
    def calculate_init_accelerations(ego_id,
                                    num_vehcl,
                                    velocities,
                                    desired_v,
                                    delta_v,
                                    delta_dist):    
        
        accelerations = np.zeros((num_vehcl, 1))
        
        for vehcl_id in range(num_vehcl):
            if vehcl_id!=ego_id:
                accelerations[vehcl_id] = AIController.IDM(velocities[vehcl_id],
                                                    desired_v[vehcl_id],
                                                    delta_v[vehcl_id],
                                                    delta_dist[vehcl_id])

        return accelerations
    
    ###########################################################################
    ###########################################################################