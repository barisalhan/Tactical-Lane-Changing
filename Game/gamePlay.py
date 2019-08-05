# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 12:02:26 2019

@author: Baris ALHAN
"""

from Game.gameMode import gameMode
from Game.gameDynamics import gameDynamics

from Vehicle.vehicle import vehicle

from Display.display import display

import numpy as np
import pygame,pdb
from pygame.locals import *

# TODO: add all vehicle related variables to the vehicle class.
# TODO: add reset method.
# TODO: make [x_pos, y_pos]
class gamePlay:
    
    '''
        Controls the flow of the game.
        
        Velocity unit is m/s.
        Time unit is second.
    '''

   
    def __init__(self):
        '''
            Initalizes all necessary modules to start the game.
        '''
        #: int: Time of the simulation (s)
        self._time = 0
        #: float: Analog of the real time required to do just one step (s)
        self._dt = 0.05

        # The below constructors are created with default parameters,
        # to read about the parameters of a class, go to the related class.
        self._mode = gameMode()
        self._dynamics = gameDynamics()
        self._display = display(self)


        #######################################################################
        #####                           VEHICLES                          #####
        #######################################################################
        #: int: Id of the ego vehicle, it is always at the median index.
        self._ego_id = int((self._dynamics._num_veh - 1) / 2)

        #: ndarray[LaneID, X_pos]: Position of the each vehicle 
        self._vehcl_positions = vehicle.generate_init_positions(self._ego_id,
                                                                self._dynamics._num_veh,
                                                                self._dynamics._num_lane,
                                                                self._display._window_width)
        #: float: It is used to calculate whether the goal distance is reached or not.
        self._init_x_point_of_ego = self._vehcl_positions[self._ego_id, 1]
        #: ndarray of flaot: Velocities of the each vehicle (m/s)
        self._velocities = vehicle.generate_init_velocities(self._ego_id,
                                                            self._dynamics._num_veh)
        
        #: ndarray of float: Stores the desired max. velocities for each vehicle.
        self._desired_v = vehicle.calculate_desired_v(self._ego_id,
                                                      self._dynamics._num_veh,
                                                      self._dynamics._desired_min_v,
                                                      self._dynamics._desired_max_v)

        #: ndarray of float, ndarray of float: Stores the velocity and distance
        #  differences with the front vehicle for each vehicle.
        self._delta_v, self._delta_dist = vehicle.calculate_deltas(self._vehcl_positions,
                                                                   self._velocities,
                                                                   self._dynamics._num_veh,
                                                                   self._dynamics._num_lane)
        #: ndarray of float: Accelerations according to the control model.
        self._accelerations = np.zeros((self._dynamics._num_veh))
        
        #: list of vehicle: Stores vehicle objects
        self._vehicles = self.create_vehicles()
        #######################################################################
        #######################################################################
        
        self._display.env_init()
        
    ###########################################################################
    
    def create_vehicles(self):
        
        vehicles = []
        
        for vehcl_id in range(self._dynamics._num_veh):
            if vehcl_id!=self._ego_id:
                vehicles.append(vehicle(self, vehcl_id, False))
            else:
                vehicles.append(vehicle(self, vehcl_id, False))
                
        return vehicles
    
    
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
        # !!DEPRECATED!!
        decisions = np.zeros((self._dynamics._num_veh, 1))
        # We can think gains list as the priority level of each vehicle
        # to change the lane. During the execution, the vehicle with top priority
        # will change the lane first.
        # !!DEPRECATED!!
        gains = np.zeros((self._dynamics._num_veh, 1))
        # Holds the target lane after lane change decision is made for each vehicle.
        # !!DEPRECATED!!
        target_lanes = self._vehcl_positions[:, 0]
        # To detect collisions, the estimated time of collision is calculated.
        # The low and high range are used to determine the difference between
        # the near and hard collision.
        TIME_OF_COLLISION_LOW = 0.1
        TIME_OF_COLLISION_HIGH = 1.8
        
        # TODO: Currently, they can only change the lane just once.
        # AIControlller calculates acceleration and lane change decision.
        for vehcl in self._vehicles:
            if vehcl._is_ego != True:
                vehcl._AIController.control()
            else:
                pass                
            
            if vehcl._is_lane_changing:
                # Update the position and heading angle.
                self._vehcl_positions[vehcl._id, 1], \
                self._vehcl_positions[vehcl._id, 0], \
                vehcl._psi = vehcl.lane_change(self._vehcl_positions[vehcl._id],
                                               vehcl._psi,
                                               self._velocities[vehcl._id],
                                               vehcl._target_lane)
            else:
               # Update the x-position of non-lane-changing vehicle.
               self._vehcl_positions[vehcl._id, 1] = self._vehcl_positions[vehcl._id, 1] + (
                                                      self._velocities[vehcl._id] * self._dt)      
        
        # Updating the velocities of the vehicles
        self._velocities = self._velocities + (self._accelerations * self._dt)
        # Updating the time of the simulation
        self._time = self._time + self._dt
        # Updating the visual environment of the simulation
        self._display.env_update()
        
        '''
        #######################################################################
        ######          Evaluating the decision of the Ego Vehicle       ######
        #######################################################################
        if action != 0.0:
            # There is a punishment to reward the less-step-required action.
            reward += -0.1

            target_lanes[
                self._ego_id] = self._vehcl_positions[self._ego_id, 0] + action

            # If ego vehicle decided to change the lane
            iterate = True

            while iterate:
                # Safety check
                if target_lanes[self._ego_id] not in range(
                        0, self._dynamics._num_lane):
                    target_lanes[self._ego_id] = self._vehcl_positions[
                        self._ego_id, 0]
                    action = 0
                    reward += -1
                    iterate = False

                x_pos = self._vehcl_positions[:, 1:]
                y_pos = self._vehcl_positions[:, 0:1]
                psi = np.zeros((self._dynamics._num_veh))

                [_, y_pos, psi, _] = self._AIController.lane_change(
                    x_pos, y_pos, psi, self._velocities, target_lanes)

                for id in range(0, self._dynamics._num_veh):
                    self._vehcl_positions[
                        id, 0] = self._AIController.round_with_offset(
                            y_pos[id], decisions[id])

                self._delta_v, self._delta_dist = self.calculate_deltas(
                    self._vehcl_positions, self._velocities)
                ### INCLUDES ALL VEHICLES END**********************************

                time_of_collision = np.divide(self._delta_dist,
                                              self._delta_v + 0.01)

                if len(time_of_collision[(
                        time_of_collision >= TIME_OF_COLLISION_LOW) & (
                            time_of_collision < TIME_OF_COLLISION_HIGH)]) > 0:
                    near_collision = 1

                # TODO: Ask is it okey to detect hard collision by this method
                if len(time_of_collision[(time_of_collision <
                                          TIME_OF_COLLISION_LOW)]) > 0:
                    hard_collision = 1
                    is_done = True
                    iterate = False

                observation = self.get_input_states(
                    self._vehcl_positions[:, :], self._velocities[:],
                    self._time)

                self._display.env_update()
                self._display._main_clock.tick()
                pygame.event.pump()

                if iterate:
                    if abs(target_lanes[self._ego_id] -
                           self._vehcl_positions[self.ego_veh_id, 0]) < 0.1:
                        action = 0
                        iterate = False

                if np.sum(abs(accelerations)) < 0.05:
                    is_done = True

            self.distance_traveled = self._vehcl_positions[self._ego_id,
                                                           1] - self._init_x_point_of_ego

            if self.distance_traveled >= self._GOAL_DISTANCE:
                reward += 10
                is_done = True
            if hard_collision:
                reward = -10
            else:
                reward += (-near_collision) * 5
                reward += 100 * (self._velocities[self._ego_id] - 10
                                 ) / self._desired_v[self._ego_id]
                return observation, reward, is_done, {}
        '''
        #######################################################################
        #######################################################################
