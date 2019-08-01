# -*- coding: utf-8 -*-
"""
Created on Tue Jul  2 10:38:11 2019

@author: Baris ALHAN
"""
import math
import numpy as np

from Vehicle.vehicleControlModels import PID
from Vehicle.vehicleControlModels import dynModel as DynModel


'''
    AI control unit for the agents in the game.
    
    IDM is used for calculating acceleration.
    MOBIL is used for deciding the lane change movement.
    
'''
class vehicleAIController: 
    
    def __init__(self, vehicle): 
        
        self._vehicle = vehicle
        self._game = self._vehicle._game
        
        self._id = self._vehicle._id
        
        self._is_lane_changing = False
        
        #  0 => Go straight
        #  1 => Go to the left
        # -1 => Go to the right
        self._lane_change_decision = 0
        
        self._target_lane = -1
        
        self._psi = 0
    
    
    # Two-Point Visual Control Model of Steering
    # For reference, please check the paper itself.
    # Perform lane change in continuous space with using controller
    # psi -> heading angle.
    def lane_change(self, x_pos_current, y_pos_current, psi_current, v_current, target_lane):
        
        pid = PID()
        dynModel = DynModel()
        
        dify = target_lane - y_pos_current
        print(type(dify))
        # two points are seleceted within 5 meters and 100 meters, then angles are calculated and fed to PID
        near_error = (math.atan2(dify, 5) - psi_current)
        far_error = (math.atan2(dify, 100) - psi_current)
        pid_out = pid.update(near_error, far_error, self._game._dt)
        u = pid_out

        z = [x_pos_current, y_pos_current, psi_current, v_current]

        [x_next, y_next, psi_next, v_next] = dynModel.update(z, u)

        return [x_next, y_next, psi_next, v_next]
     
    # when to decide that lane changes has finished!
    def round_with_offset(self, value, dec):					
        rounded_value = abs(np.round(value))					   
        rounded_value[dec == 1] = abs(np.round(value[dec == 1] + 0.30))			 
        rounded_value[dec == -1] = abs(np.round(value[dec == -1] - 0.30)) 
        return rounded_value

    
    # TODO: check whether it is my own vehicle or not.
    # TODO: Write binary search here.
    def find_follower_vehicle(self, position):
        
        min_dist = 99999999
        result_id = -1
        
        vehcl_id = 0
        for coordinate in self._game._vehcl_positions:
            if coordinate[0] == position[0]:
                if position[1] - coordinate[1] > 0:
                    if position[1] - coordinate[1] < min_dist:
                        min_dist = position[1] - coordinate[1]
                        result_id = vehcl_id
            vehcl_id += 1
        
        return result_id
        
    
    # TODO: check is there any calculation made in the step related to the deltas.
    def calculate_acceleration(self, vehcl_id, new_coordinates):
        
        new_delta_v, new_delta_dist = self._vehicle.calculate_deltas(new_coordinates,
                                                                     self._game._velocities,
                                                                     self._game._dynamics._num_veh,
                                                                     self._game._dynamics._num_lane)
            
        new_acceleration = vehicleAIController.IDM(self._game._velocities[vehcl_id],
                                                   self._game._desired_v[vehcl_id],
                                                   new_delta_v[vehcl_id],
                                                   new_delta_dist[vehcl_id])
        return new_acceleration    
    
    
    # Return a number that represents the possible gain from that 
    # lane change movement
    def check_incentive_criterion(self, movement):
        
        gain = 0
        
        p = 1
        q = 0.5
        
        if self._game._mode._rule_mode == 1:

            positions = self._game._vehcl_positions
            new_positions = np.copy(self._game._vehcl_positions)
            new_positions[self._id, 0] = new_positions[self._id, 0] + movement                
            
            rear_vehcl_id = self.find_follower_vehicle(positions[self._id])
            
            new_rear_vehcl_id = self.find_follower_vehicle(new_positions[self._id])
            
            self_acc = self._game._accelerations[self._id]
        
            self_new_acc = self.calculate_acceleration(self._id, new_positions)
            
            rear_vehcl_acc = vehicleAIController.IDM(self._game._velocities[rear_vehcl_id],
                                                     self._game._desired_v[rear_vehcl_id],
                                                     self._game._delta_v[rear_vehcl_id],
                                                     self._game._delta_dist[rear_vehcl_id])
            
            rear_vehcl_new_acc = self.calculate_acceleration(rear_vehcl_id, new_positions)
            
            new_rear_vehcl_acc = vehicleAIController.IDM(self._game._velocities[new_rear_vehcl_id],
                                                         self._game._desired_v[new_rear_vehcl_id],
                                                         self._game._delta_v[new_rear_vehcl_id],
                                                         self._game._delta_dist[new_rear_vehcl_id])
            
            new_rear_vehcl_new_acc = self.calculate_acceleration(new_rear_vehcl_id, new_positions)
            
            gain = ((self_new_acc - self_acc) + 
                        p*(rear_vehcl_new_acc - rear_vehcl_acc) +
                            q*(new_rear_vehcl_new_acc - new_rear_vehcl_acc))
                        
            return gain
        elif  self._game._mode._rule_mode == 2:
            pass
        
        
    def check_safety_criterion(self, movement):
        
        #: flaot: Maximum safe deceleration (m/s^2)
        bsafe = -4.0 
        
        new_positions = np.copy(self._game._vehcl_positions)
        new_positions[self._id, 0] = new_positions[self._id, 0] + movement
        new_lane = new_positions[self._id][0]        
                   
        # Checks whether the lane change movement takes vehicle out of the road.
        if new_lane >= self._game._dynamics._num_lane or new_lane < 0:
                return False  
        
        follower_vehcl_id = self.find_follower_vehicle(new_positions[self._id])
        
        follower_vehcl_new_acc = self.calculate_acceleration(follower_vehcl_id,
                                                                 new_positions)
            
        if follower_vehcl_new_acc < bsafe:
            return False
        
        return True
    
        
    '''
    There are three movements in terms of lane changing:
         0 => Go straight
         1 => Go to the left
        -1 => Go to the right
    
    Algorithm: 
        Calculate the gains of all possible lane change movements.
        According to gains decide which one to select.
    '''        
    def MOBIL(self):
        
        decision = 0
        
        # Holding the possible gains for each possible lane change movement.
        # gains[0] => go straight
        # gains[1] => go to the left
        # gains[2] => go to the right
        gains = []
        
        # movement represents to go straight, left or right respectively.
        for movement in range(0,3):
            if self.check_safety_criterion(movement):
                gains.append(self.check_incentive_criterion(movement))
            else:
                gains.append((-999))
       
        decision = gains.index(max(gains))
        
        if decision == 2:
            decision = -1
        
        return decision
    
    
    '''
        One of the main functions for traffic simulation.
        It controls the longitudinal accelerations of vehicles.
        For reference, please check the paper itself.
        
        Inputs:
            velocity   : current speed of the vehicle
            desired_v   : desired speed of the vehicle
            delta_v     : Speed diffrence with the leading vehicle
            delta_dist  : Gap with the leading vehicle
        Outputs: 
            acceleration : Reference Acceleration, that value used for lane change safety check  
    '''
    @staticmethod
    def IDM(velocity,
            desired_v,
            delta_v,
            delta_dist):
        
        amax = 0.7  # Maximum acceleration    (m/s^2) 
        S = 4  # Acceleration exponent
        d0 = 2  # Minimum gap
        T = 1.6  # Safe time headaway    (s)
        b = 1.7  # Desired deceleration (m/s^2)
        
        dstar = d0 + (velocity * T) + ((velocity * delta_v) / (2 * math.sqrt(amax * b)))
        acceleration = amax * (1 - math.pow( ( velocity/desired_v ), S) - math.pow( (dstar/(delta_dist + 0.001)) , 2) )
    
        # Lower bound for acceleration, -20 m/s^2
        if acceleration < -20:
            acceleration = -20  
        
        return acceleration
    
    
    def control(self):
        
        acceleration = vehicleAIController.IDM(self._game._velocities[self._id],
                                               self._game._desired_v[self._id],
                                               self._game._delta_v[self._id],
                                               self._game._delta_dist[self._id])
            
        self._game._accelerations[self._id] = acceleration
        
        # Check if the traffic rule enables lane changing.
        if self._game._mode._rule_mode !=0:
            if self._is_lane_changing==False:
                self._lane_change_decision = self.MOBIL()        
        
       
       