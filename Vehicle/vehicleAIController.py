# -*- coding: utf-8 -*-
"""
Created on Tue Jul  2 10:38:11 2019

@author: Baris ALHAN
"""
import math,pdb
import copy
import numpy as np


'''
    AI control unit for the agents in the game.
    
    IDM is used for calculating acceleration.
    MOBIL is used for deciding the lane change movement.
    
    Possible optimizations and protocols to make traffic flow more safe:
        Safety must be the first incentive rather than checking just acceleration
    of previous and next conditions. However, while checking safety, it must be
    parametrized because in some situations, driver have to decide between
    to bad decisions and he have to choose to one that will survive him longer.
        Incentive comes after, and the politeness factor must be added to there.
    However, there is one big problem, since the velocity and acceleration values are
    far from the real values testing gets harder and harder.
        Process of tuning variables must be parametrized.
        Generation of the same scenario must be enabled, and it will simplfy the
    testing process a lot.
        Genetic algorithm could be used in automatized tuning of the variables.    
'''
class vehicleAIController: 
    
    def __init__(self, vehcl, vehicles, mode, dynamics): 
        
        self._vehcl = vehcl
        self._vehicles = vehicles
        self._mode = mode
        self._dynamics = dynamics
        
        self._id = self._vehcl._id
    
    
    # TODO: check is there any calculation made in the step related to the deltas.
    def calculate_acceleration(vehcl_v,
                               vehcl_pos,
                               vehcl_desired_v,
                               front_vehcl_v,
                               front_vehcl_pos):
        
        new_delta_v = vehcl_v - front_vehcl_v
        new_delta_dist = front_vehcl_pos[1] - vehcl_pos[1]
            
        new_acceleration = vehicleAIController.IDM(vehcl_v,
                                                   vehcl_desired_v,
                                                   new_delta_v,
                                                   new_delta_dist)
        return new_acceleration   
    
    '''
        Returns a number that represents the possible
        gain from lane change movement.
        
        Algorithm:
            1. Find new rear vehicle
            2. Calculate the acceleration of the new rear vehicle before lane change.
            3. Calculate the acceleration of the new rear vehicle after lane change.
            4. Calculate the acceleration of the self vehicle.
            5. Calculate the acceleration of the self vehicle after lane change.
            6. Calculate the gain.
    '''
    def check_incentive_criterion(self, movement):
        
        gain = 0
        
        p = 1
        
        if self._mode._rule_mode == 1:
            
            new_position = (self._vehcl._position[0] + movement,
                            self._vehcl._position[1])
            
            new_rear_vehcl = vehicleAIController.find_rear_vehicle(self._vehicles,
                                                                   new_position)
            
            new_rear_vehcl_acc_before = 0
            new_rear_vehcl_acc_after = 0
            
            if new_rear_vehcl and new_rear_vehcl._is_lane_changing == False:                
                new_rear_vehcl_acc_before = vehicleAIController.IDM(new_rear_vehcl._velocity,
                                                                    new_rear_vehcl._desired_v,
                                                                    new_rear_vehcl._delta_v,
                                                                    new_rear_vehcl._delta_dist)
                
                new_rear_vehcl_acc_after = vehicleAIController.calculate_acceleration(new_rear_vehcl._velocity,
                                                                                      new_rear_vehcl._position,
                                                                                      new_rear_vehcl._desired_v,
                                                                                      self._vehcl._velocity,
                                                                                      self._vehcl._position)
            acc = self._vehcl._acceleration
            
            new_front_vehcl = vehicleAIController.find_front_vehicle(self._vehicles,
                                                                     new_position)
            if new_front_vehcl and new_front_vehcl._is_lane_changing == False:
                acc_new = vehicleAIController.calculate_acceleration(self._vehcl._velocity,
                                                                     new_position,
                                                                     self._vehcl._desired_v,
                                                                     new_front_vehcl._velocity,
                                                                     new_front_vehcl._position)
            else:
                acc_new = vehicleAIController.IDM(self._vehcl._velocity,
                                                  self._vehcl._desired_v,
                                                  0,
                                                  100000)
            
            
            gain_self = (acc_new - acc)  
            gain_rear = p*(new_rear_vehcl_acc_after - new_rear_vehcl_acc_before)
            
            print("id:{},  gain_self:{}, gain_rear:{}".format(self._id,gain_self, gain_rear))
            
            if gain_self < 2:
                gain_self = 0
                
            if gain_rear > -3 and gain_rear < 3:
                gain_rear = 0
                
            gain = gain_self + gain_rear
       
            return gain
         
        elif  self._mode._rule_mode == 2:
            pass

    '''
        Algorithm:
            1. Check whether lane change movement is valid.
               After now, I will explain as if the lane change movement is done,
               and the vehicle is in the target lane.
            3. Find the rear vehicle.
            4. Find the acceleration of the rear vehicle.
            5. Fidn the front vehicle.
            6. Find the acceleratin of the self vehicle.
            7. Check whether accelerations are in safe range or not.
            8. Find the side vehicle and check whether it is too close.
    '''
    # TODO: they are going out of the road.
    def check_safety_criterion(self, movement):

        #: flaot: Maximum safe deceleration (m/s^2)
        bsafe = -4.0
        
        position = (self._vehcl._position[0] + movement,
                            self._vehcl._position[1])
        
        lane = position[0]
       
        # Checks whether the lane change movement takes vehicle out of the road.
        if (lane > self._dynamics._num_lane - 1) or (lane < 0):
                print("A")
                return -9999
        
        '''
        rear_vehcl = vehicleAIController.find_rear_vehicle(self._vehicles,
                                                           position)
        if rear_vehcl:
            rear_vehcl_acc = vehicleAIController.calculate_acceleration(rear_vehcl._velocity,
                                                                        rear_vehcl._position,
                                                                        rear_vehcl._desired_v,       
                                                                        self._vehcl._velocity,
                                                                        position)
            if rear_vehcl_acc < bsafe:
                print("B")
                return False
        '''
            
        # Checks whether the side vehicle in the target lane is too close.    
        if movement!=0:
            tmp_pos = (position[0], position[1] - 0.1)
            side_vehcl_front = vehicleAIController.find_front_vehicle(self._vehicles,
                                                                      tmp_pos)
            tmp_pos = (position[0], position[1] + 0.1)
            side_vehcl_rear = vehicleAIController.find_rear_vehicle(self._vehicles,
                                                                    tmp_pos)
            
            safety_distance = 2.4
            if side_vehcl_front:
                if side_vehcl_front._position[1] - position[1] < safety_distance:
                    print("D")
                    return -8999
            if side_vehcl_rear:
                if position[1] - side_vehcl_rear._position[1] < safety_distance:
                    print("E")
                    return -8999
        
        front_vehcl = vehicleAIController.find_front_vehicle(self._vehicles,
                                                             position)
        if front_vehcl:
            acc = vehicleAIController.calculate_acceleration(self._vehcl._velocity,
                                                             position,
                                                             self._vehcl._desired_v,
                                                             front_vehcl._velocity,
                                                             front_vehcl._position)
            if acc < bsafe:
                print("C")
                return -100
            
        return 5
    
        
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
                
            gains.append(self.check_safety_criterion(movement) +
                         self.check_incentive_criterion(movement))
       
        decision = gains.index(max(gains))
        
        for x in gains:
            print(x)
        print("------")
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
            acceleration : Reference Acceleration
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

        self._vehcl._acceleration = vehicleAIController.IDM(self._vehcl._velocity,
                                                            self._vehcl._desired_v,
                                                            self._vehcl._delta_v,
                                                            self._vehcl._delta_dist)
        #x = vehicleAIController.find_follower_vehicle(self._vehicles,
        #                                              self._vehcl._position)
        #if x:
        #    print("id:{}, follower{}".format(self._id,x._id))
        
        # Checks if the traffic rule enables lane changing.
        if self._mode._rule_mode !=0:
            if self._vehcl._is_lane_changing==False:
                self._vehcl._lane_change_decision = self.MOBIL()   
                #print("id:{}, decision:{}".format(self._id, self._vehcl._lane_change_decision))   
                if self._vehcl._lane_change_decision!= 0:
                    self._vehcl._is_lane_changing = True
                    self._vehcl._target_lane = (self._vehcl._position[0] + 
                                                self._vehcl._lane_change_decision)
             
                   
    ###########################################################################
    ######                    STATIC METHODS                              #####
    ########################################################################### 
                   
    # TODO: find functions are not optimal.
    @staticmethod
    def find_rear_vehicle(vehicles, position):
        
        min_dist = 99999999
        result_vehcl = None
        result_id = -1
        
        vehcl_id = 0    
        for vehcl in vehicles:
            if abs(vehcl._position[0] - position[0]) < 0.7 :
                if position[1] - vehcl._position[1] > 0.0001:
                    if position[1] - vehcl._position[1] < min_dist:
                        min_dist = position[1] - vehcl._position[1]
                        result_id = vehcl_id
            vehcl_id += 1
        
        if result_id!=-1:
            result_vehcl = vehicles[result_id]
            
        return result_vehcl
    
    
    @staticmethod
    def find_front_vehicle(vehicles, position):
        
        min_dist = 99999999
        result_vehcl = None
        result_id = -1
        
        vehcl_id = 0    
        for vehcl in vehicles:
            if abs(vehcl._position[0] - position[0]) < 0.7 :
                if  vehcl._position[1] - position[1] > 0.0001:
                    if  vehcl._position[1] - position[1] < min_dist:
                        min_dist = vehcl._position[1] - position[1]
                        result_id = vehcl_id
            vehcl_id += 1
        
        if result_id!=-1:
            result_vehcl = vehicles[result_id]
        
        return result_vehcl
    
    ###########################################################################
    ###########################################################################       