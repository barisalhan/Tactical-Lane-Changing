# -*- coding: utf-8 -*-
"""
Created on Tue Jul  2 10:38:11 2019

@author: Baris ALHAN
"""
import math,pdb
import numpy as np


'''
    AI control unit for the agents in the game.
    
    IDM is used for calculating acceleration.
    MOBIL is used for deciding the lane change movement.
    
'''
class vehicleAIController: 
    
    def __init__(self, vehcl): 
        
        self._vehcl = vehcl
        self._game = self._vehcl._game
        
        self._id = self._vehcl._id
    
    # TODO: check is there any calculation made in the step related to the deltas.
    def calculate_acceleration(self, vehcl):
        
        new_delta_v, new_delta_dist = self._vehcl.calculate_deltas(self._game,
                                                                   vehcl)
        
        new_acceleration = vehicleAIController.IDM(vehcl._velocity,
                                                   vehcl._desired_v,
                                                   vehcl.new_delta_v,
                                                   vehcl.new_delta_dist)
        return new_acceleration    
    
    
    # Return a number that represents the possible gain from that 
    # lane change movement
    def check_incentive_criterion(self, movement):
        
        gain = 0
        
        p = 1
        q = 0.5
        
        if self._game._mode._rule_mode == 1:

            new_positions = []
            for vehcl in self._game._vehicles:
                new_positions.append(vehcl._position)
                
            new_positions[self._id, 0] = new_positions[self._id, 0] + movement                
            
            rear_vehcl_id = self.find_follower_vehicle(self._vehcl._position)
            
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
        
        new_positions = []    
        for vehcl in self._game._vehicles:
            new_positions.append(vehcl._position)
            
        new_positions[self._id, 0] = new_positions[self._id, 0] + movement
        new_lane = new_positions[self._id][0]        
                   
        # Checks whether the lane change movement takes vehicle out of the road.
        if new_lane >= self._game._dynamics._num_lane or new_lane < 0:
                return False  
        
        follower_vehcl = self.find_follower_vehicle(self._game,
                                                    new_positions[self._id])
        
        follower_vehcl_new_acc = self.calculate_acceleration(follower_vehcl,
                                                             new_positions)
        self_vehcl_new_acc = self.calculate_acceleration(self._id,
                                                         new_positions)
        
        if follower_vehcl_new_acc < bsafe:
            return False
        
        if self_vehcl_new_acc < bsafe:
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
        # gains[1] => go to the right
        # gains[2] => go to the left
        gains = []
        
        # movement represents to go straight, left or right respectively.
        for movement in range(0,3):
            
            if movement==2:
                movement = -1
                
            if self.check_safety_criterion(movement) == True:
                gains.append(self.check_incentive_criterion(movement))
            else:
                gains.append((-9999))
       
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
        #pdb.set_trace()
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
        
        self._vehcl._delta_v,\
        self._vehcl._delta_dist = self._vehcl.calculate_deltas(self._game,
                                                               self._vehcl)
        
        #print("id:{}, delta_v:{}, delta_dist:{}".format(self._vehcl._id,self._vehcl._delta_v,self._vehcl._delta_dist))

        self._vehcl._acceleration = vehicleAIController.IDM(self._vehcl._velocity,
                                                            self._vehcl._desired_v,
                                                            self._vehcl._delta_v,
                                                            self._vehcl._delta_dist)
        
        # TODO: You're trying to enable the lane changing.
        '''               
        # Check if the traffic rule enables lane changing.
        if self._game._mode._rule_mode !=0:
            if self._vehcl._is_lane_changing==False:
                self._vehcl._lane_change_decision = self.MOBIL()    
                if self._vehcl._lane_change_decision!= 0:
                    self._vehcl._is_lane_changing = True
                    self._vehcl._target_lane = (self._vehcl._position[0] + 
                                                self._vehcl._lane_change_decision)
       '''
       
       