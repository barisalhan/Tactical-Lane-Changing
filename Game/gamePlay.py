# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 12:02:26 2019

@author: Baris ALHAN
"""
import Vehicle.VehicleControlModel
import Vehicle.VehicleDynamics

import numpy as np
import pygame 
from pygame.locals import *

class gamePlay:
    '''
        The class that actions of the game happens. We could think this
        class as the brain of the entire project.
        
        All the velocity calculations are made in the unit of m/s.
        
        # TODO: gym will be connected to here.
    '''
    # TODO: add reset method.
    def __init__(self, mode, dynamics, veh_props, veh_model):

        #######################################################################
        #####                       INITIALIZATION                        #####
        #######################################################################
        self._mode = mode
        self._dynamics = dynamics
        self._veh_props = veh_props
        self._veh_model = veh_model
        #######################################################################
        #######################################################################
        
        # TODO: think about whether it is necessary or not.
        # time tick of the simulation (s)
        self._time = 0
        
        
        self._window_width = int(self._dynamics._max_veh_inlane * 
                               (self._veh_props._width + 2 * self._veh_props._height))
        self._window_height = int((self._veh_props._height +  2 * (self._veh_props._height // 2.5)) * self._dynamics._num_lane )

        
        #######################################################################
        #####                           VEHICLES                          #####
        #######################################################################
        # The id of the ego vehicle.
        self._ego_id = int((self._dynamics._num_veh-1)/2)
        
        # Coordinates of the each vehicle [LaneID : X_pos]
        self._veh_coordinates = self.generate_init_points()
        # Velocities of the each vehicle (m/s)
        self._init_velocities = self.generate_init_velocities()
        
        # The velocity of the ego vehicle (m/s)
        self._ego_v = self._init_velocities[self._ego_id]
        
        # The list that stores the desired max. velocities for the each vehicle 
        self._desired_v = self.calculate_desired_v(self._dynamics._desired_min_v,
                                                            self._dynamics._desired_max_v)
        
        # The lists that stores the velocity and distance differences with the front vehicle for each vehicle
        self._delta_v, self._delta_dist = self.generate_deltas(self._veh_coordinates, self._init_velocities)
        # The list holds the velocities of vehicles throughout the program.
        self._current_velocities = self._init_velocities
        
        self._mobil_mask = np.random.random_integers(0, 1, size=self._dynamics._num_veh)
        #######################################################################
        #######################################################################
        
        
        
        
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
   

    
    
    # TODO: understand what's going on here.
    #def play(self):
    #    metadata = {'render.modes':['human']}
      
    
    '''
        One of the main functions for traffic simulation. It controls the longitudinal accelerations for each vehicle.
        For reference, please check the paper itself.
        
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
 


    '''
     The method generates the initial positions of vehicles.
     
     Aim: Vehicles are distributed to the highway without collisions.
     
     The algorithm is as follows:
         1. Assign each vehicle to a free lane
            For each lane evaluate the exact position of each vehicle.
             2. For each lane in lanes:
                 3. First, randomly calculate a point
                    for the first car in that lane.
                 4. Afterwards, assign a position to the remanining
                    vehicles one by one, ensuring that they do not collide.
     
     Inputs: init_range, delta_dist
         init_range : range of the initialization horizon(meters)
         delta_dist: minimum distance between vehicles (meters)
         
     Outputs: coordinates, ego_veh_id, init_v, lane_ids
         coordinates : Position of each vehicle
         init_v      : Initial speed for each vehicle
    '''
    # TODO: Check the safety of delta_dist
    # TODO: Check the necessity of init_v
    # TODO: comment the lane IDs
    # TODO: remove the creating initial velocities part from here. Make another method.
    # TODO: velocities are unnecessary!
    def generate_init_points(self, init_range = 200, delta_dist = 25):
        
        #The result list stores the lane of each vehicle.
        lane_list = []
        #The result list stores the coordinates of each vehicle.
        #[LaneID, X_pos)]
        coordinates = np.zeros((self._dynamics._num_veh, 2))
        
               
        #first randomly select lanes for each vehicle
        for car in range(0, self._dynamics._num_veh):
            # Randomly chose lane id for each vehicle
            lane_list.append(np.random.randint(0, self._dynamics._num_lane))  
        
        #The map that stores [LaneID <-> number of cars in that lane]
        fullness_of_lanes = {x: lane_list.count(x) for x in lane_list}        
        
        # Temporary list to store the coordinates of the each vehicle.
        # [(LaneID : X_position)]
        tmp_coordinates = []
        
        # 2nd step of the algorithm.
        for lane, num_vehicles_inlane in fullness_of_lanes.items():
            # Temporary point list in the x direction
            # to store the positions of the vehicles.
            tmp_points = list([])
            # First, chose a point for the first vehicle in the selected lane
            # The second parameter in the randomness ensures that there is no
            # accumulation of cars at the end of the inital range.
            tmp_points.append(np.random.uniform(0 , init_range - ((num_vehicles_inlane - 1) * delta_dist) ))
            
            tmp_coordinates.append([lane, tmp_points[-1]])
            for car_id in range(0, num_vehicles_inlane - 1):
                # put other vehicles in that lane to the remaining space
                tmp_points.append(np.random.uniform(tmp_points[-1] + delta_dist,
                                                    (init_range - (num_vehicles_inlane - 2 - car_id) * delta_dist)))
                tmp_coordinates.append([lane, tmp_points[-1]])
                
        coordinates = np.asarray(tmp_coordinates).reshape(coordinates.shape)
        coordinates = coordinates[coordinates[:, 1].argsort()]
        # TODO : Ask what's going on here!
        coordinates[:, 1] = coordinates[:, 1] - coordinates[self._ego_id, 1] + self._window_width / 20
    
        return coordinates  
    
    def generate_init_velocities(self):
        
        #The result list stores the initial velocities of each vehicle.
        init_v = np.zeros((self._dynamics._num_veh, 1))
        
        # initial velocities for the ego vehicle => 10m/s 15 m/s
        # TODO: parametrize here!
        # TODO: AT THE END: check whether here is necessary or not.(Is there any redundancy?)
        init_v[self._ego_id] = np.random.uniform(10, 15)

        for rear_id in range(0, self._ego_id):
             # 26.4, 33.3  , randomly define initial speeds for rear vehicles  
            init_v[rear_id] = np.random.uniform(15, 25) 
        for front_id in range(self._ego_id + 1, self._dynamics._num_veh):
             # 16.7, 23.6 , randomly define initial speeds for front vehicles
            init_v[front_id] = np.random.uniform(10, 12) 
        
        return init_v
    
    
    # TODO: ask what's going on here.
    # Calucate delta_v and delta_dist.
    def generate_deltas(self, current_states, current_v):
        
        num_veh = self._dynamics._num_veh
        num_lane = self._dynamics._num_lane
        
        delta_v = np.zeros((num_veh, 1))
        delta_dist = np.zeros((num_veh, 1))
        
        for lane_id in range(num_lane):
            
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
    
    # TODO: understand what's going on here!
    # TODO: explain the general behavior of the method.
    # this is for RL agents
    def step(self, action):
        
        isDone = False
        
        Near_Collision = False
        Hard_Collision = False
        
        # TODO: ??
        rew = np.array([0.])
        
        decisions = np.zeros((self._dynamics._num_car, 1))
        
        gains = np.zeros((self._dynamics._num_car, 1))
        
        # TODO: YOU ARE HERE!
        V_dot, X_dot, _ = self.idm(self._current_velocities, self._desired_v,
                                   self._delta_v, self._delta_dist)
        
        # TODO: parametrize it.
        if self._mode._behavior_mode == "full_mobil":
            self._mobil_mask = np.ones(self._dynamics._num_veh)

        # If traffic rule is USA or UK.
        if self._mode._rule_mode == 1 or self._mode._rule_mode == 2:
            if self._time > 0:
                for id in range(0, self._dynamics._num_veh):
                    ll_id, lf_id, ml_id, mf_id, rl_id, rf_id = self.check_car_id(self.States, idx)
                    decisions[id], gains[id] = self.check_safety(id, self.States, V_dot, self.V, v_d, ll_id,
                                                                 lf_id,
                                                                 ml_id, mf_id, rl_id, rf_id)

            for idx in range(0, NoOfCars):
                if decision[idx] != 0 and idx == np.argmax(gain):
                    self.target_lane[idx] = self.States[idx, 0] + decision[idx]
                    if self.target_lane[idx] not in range(0, NoOfLanes):
                        self.target_lane[idx] = self.States[idx, 0]
            self.target_lane[self.ego_veh_id] = self.States[self.ego_veh_id, 0]
        if action == 2:
            rew += -1
            action = decision[self.ego_veh_id]
            # for idx in range(0, NoOfCars):
            #     if idx != self.ego_veh_id:
            #         decision[idx] = 0

            # decision[self.ego_veh_id] = 0

        if action != 0.0:
            rew += -0.1
            self.number_of_lane_change = self.number_of_lane_change + 1
            self.target_lane[self.ego_veh_id] = self.States[self.ego_veh_id, 0] + action
            iterate = True
            by_pass = False
            while iterate:
                if self.target_lane[self.ego_veh_id] not in range(0, NoOfLanes):
                    self.target_lane[self.ego_veh_id] = self.States[self.ego_veh_id, 0]
                    action = 0
                    rew += -1
                    by_pass = True
                V_dot, X_dot, _ = self.idm(self.V, v_d, self.delta_v, self.s)

                self.V = self.V + V_dot * dt
                self.States[:, 1:] = self.States[:, 1:] + X_dot * dt

                self.distance_traveled = self.States[self.ego_veh_id, 1] - self.init_pos_for_ego

                States_Continuous = np.copy(self.States)

                self.sim_time = self.sim_time + dt

                [_, self.y_pos, self.psi, _] = \
                    self.lane_change(self.States[:, 1:], self.y_pos, self.psi, self.V, self.target_lane)

                States_Continuous[:, 0:1] = self.y_pos[0:]


                for idx in range(0, NoOfCars):
                    self.States[idx, 0] = self.round_with_offset(self.y_pos[idx], decision[idx])

                self.delta_v, self.s = self.generate_deltas(self.States, NoOfCars, self.lane_id, self.V)

                time_of_collision = np.divide(self.s, self.delta_v + 0.01)
                if len(time_of_collision[(time_of_collision >= time_of_collision_low) & (
                        time_of_collision < time_of_collision_high)]) > 0:
                    Near_Collision = 1
                    # print("soft collision", self.distance_traveled)

                if len(self.s[self.s < 5]) > 0:
                    id_of_crashed_veh = np.where(self.s < 5)[0]
                    pos_of_crashed_veh = self.States[id_of_crashed_veh, 1]
                    if np.any(id_of_crashed_veh == self.ego_veh_id):
                        Hard_Collision = 1
                        # print("hard collision due to ego vehicle", self.distance_traveled)
                    elif np.any(self.States[id_of_crashed_veh, 0] == self.States[self.ego_veh_id, 0]):
                        if np.any(((pos_of_crashed_veh - 5) < self.States[self.ego_veh_id, 1]) & (
	   
	  
		   
					  
                                self.States[self.ego_veh_id, 1] < (pos_of_crashed_veh + 5))):
                            Hard_Collision = 1
                            # print("hard collision due to rear or front vehicle", self.distance_traveled)
                        else:
                            print("collision for other vehicles in same lane ")
                    else:
                        print("collision for other vehicles in different lane ")

                obs = self.get_input_states(self.States[:, :], self.V[:], self.sim_time)  # returns 27 x 1 vector

                if Hard_Collision == 1:
                    done = 1
                    iterate = False

                if PYGAME is True:
                    self.env_update(States_Continuous, self.Line_Rec_Samples, self.Emergency_Line_Rec_Samples,
                                    self.V)
                    self.MainClock.tick()
                    pygame.event.pump()

                if not by_pass:
                    if abs(self.target_lane[self.ego_veh_id] - self.y_pos[self.ego_veh_id]) < 0.1:
                        action = 0
                        iterate = False

                # delta_s_to_ego = self.States[:, 1] - self.States[self.ego_veh_id, 1]
                # # print(delta_s_to_ego, self.States[abs(delta_s_to_ego) > 100, 1], self.States[abs(delta_s_to_ego) > 100, 1] + 200)
                # self.States[delta_s_to_ego < -100, 1] = self.States[delta_s_to_ego < -100, 1] + 250
                # self.States[delta_s_to_ego > 100, 1] = self.States[delta_s_to_ego > 100, 1] - 250

                if np.sum(abs(V_dot)) < 0.05:
                    # equilibrium_counter += 1
                    done = 1
													  

                if not iterate:
                    if self.distance_traveled >= GOAL_DISTANCE:
                        rew += 10
                        done = 1

                    if Hard_Collision:
                        rew = np.array([-10.])
                    else:
                        rew += -Near_Collision * 5
                        rew += 100 * (self.V[self.ego_veh_id] - 10) / self.desired_V[self.ego_veh_id]
                    # rew += 100*np.divide(self.V[self.ego_veh_id], self.desired_V[self.ego_veh_id])
                    # print("reward:   {}".format(rew))
                        return obs, np.asscalar(rew), done, {}
                if by_pass:
                    iterate = False

        for fsi in range(int(1 / dt)):
            V_dot, X_dot, _ = self.idm(self.V, v_d, self.delta_v, self.s)

            self.V = self.V + V_dot * dt
            self.States[:, 1:] = self.States[:, 1:] + X_dot * dt

            self.distance_traveled = self.States[self.ego_veh_id, 1] - self.init_pos_for_ego

            States_Continuous = np.copy(self.States)

            self.sim_time = self.sim_time + dt

            [_, self.y_pos, self.psi, _] = \
                self.lane_change(self.States[:, 1:], self.y_pos, self.psi, self.V, self.target_lane)

            States_Continuous[:, 0:1] = self.y_pos[0:]

            for idx in range(0, NoOfCars):
                self.States[idx, 0] = self.round_with_offset(self.y_pos[idx], decision[idx])

            self.delta_v, self.s = self.generate_deltas(self.States, NoOfCars, self.lane_id, self.V)

            time_of_collision = np.divide(self.s, self.delta_v + 0.01)

            if len(time_of_collision[(time_of_collision >= time_of_collision_low) & (
                    time_of_collision < time_of_collision_high)]) > 0:
                Near_Collision = 1
                # print("soft collision", self.distance_traveled)

            if len(self.s[self.s < 5]) > 0:
                id_of_crashed_veh = np.where(self.s < 5)[0]
                pos_of_crashed_veh = self.States[id_of_crashed_veh, 1]
                if np.any(id_of_crashed_veh == self.ego_veh_id):
                    Hard_Collision = 1
                    # print("hard collision due to ego vehicle", self.distance_traveled)
                elif np.any(self.States[id_of_crashed_veh, 0] == self.States[self.ego_veh_id, 0]):
                    if np.any(((pos_of_crashed_veh - 5) < self.States[self.ego_veh_id, 1]) & (
		   
					  
                            self.States[self.ego_veh_id, 1] < (pos_of_crashed_veh + 5))):
                        Hard_Collision = 1
                        # print("hard collision due to rear or front vehicle", self.distance_traveled)
                    else:
                        print("collision for other vehicles in same lane ")
                else:
                    print("collision for other vehicles in different lane ")
            obs = self.get_input_states(self.States[:, :], self.V[:], self.sim_time)  # returns 27 x 1 vector

            if Hard_Collision == 1:
                rew = np.array([-10.])
                done = 1
            # environment is ticked here!!
            if PYGAME is True:
                self.env_update(States_Continuous, self.Line_Rec_Samples, self.Emergency_Line_Rec_Samples, self.V)
                self.MainClock.tick()
                pygame.event.pump()

            # delta_s_to_ego = self.States[:, 1] - self.States[self.ego_veh_id, 1]
            # # print(delta_s_to_ego, self.States[abs(delta_s_to_ego) > 100, 1], self.States[abs(delta_s_to_ego) > 100, 1] + 200)
            # self.States[delta_s_to_ego < -100, 1] = self.States[delta_s_to_ego < -100, 1] + 250
            # self.States[delta_s_to_ego > 100, 1] = self.States[delta_s_to_ego > 100, 1] - 250
            if done:
                return obs, np.asscalar(rew), done, {}

        if np.sum(abs(V_dot)) < 0.05:
            # equilibrium_counter += 1
            done = 1
			  
        if self.distance_traveled >= GOAL_DISTANCE:
            rew += 10
            done = 1
			

        if Hard_Collision:
            rew = np.array([-10.])
        else:
            rew += -Near_Collision * 5
            rew += 100 * (self.V[self.ego_veh_id] - 10) / self.desired_V[self.ego_veh_id]
        # rew +=  100*np.divide(self.V[self.ego_veh_id], self.desired_V[self.ego_veh_id])
        # print("reward:   {}".format(rew))
        return obs, np.asscalar(rew), done, {}

    