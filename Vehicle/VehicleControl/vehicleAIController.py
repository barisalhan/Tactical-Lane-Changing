# -*- coding: utf-8 -*-
"""
Created on Tue Jul  2 10:38:11 2019

@author: Baris ALHAN
"""
import numpy as np
from Vehicle.VehicleControlModel.PID import PID
from Vehicle.VehicleControlModel.dynModel import dynModel as DynModel

class vehicleAIController: 
    
    def __init__(self,dt,mode,dynamics):
        self._dt = dt
        self._mode = mode
        self._dynamics = dynamics
        
        self._decisions = []
    
    # TODO: implement the algorithm again!
    ## Function that returns the IDs of surrounding vehicle for each vehicle
    # mf, middle follower
    # rf, right follower 
    # lf, left follower
    # ml, middle leader
    # rl, right leader
    # ll, left leader 
    def get_surrounding_vehs(self, veh_coordinates, vehicle_id):
        # each vehicle has static unique id
        # algorithm:
        # araclarin pozisyonun gore sirala
        # aracin hangi lane'de oldugunu bul
        # en yakin araclari tespit et.
        mf, rf, lf = None, None, None
        ml, rl, ll = None, None, None
        
        sorted_ids = list(np.argsort(veh_coordinates[:, 1]))
        
        veh_lane_no = veh_coordinates[vehicle_id, 0]
        
        veh_id_sorted = sorted_ids.index(vehicle_id)
        
        trailing_cars_ids = np.argsort(veh_coordinates[:, 1])[0:veh_id_sorted]
        trailing_cars_lanes = veh_coordinates[sorted_ids[0:veh_id_sorted], 0]
        
        leader_cars_ids = np.flip(np.argsort(veh_coordinates[:, 1])[(veh_id_sorted + 1):])
        leader_cars_lanes = np.flip(veh_coordinates[sorted_ids[(veh_id_sorted + 1):], 0])
        
        for idx, lane in enumerate(trailing_cars_lanes):
            if lane == veh_lane_no:
                mf = trailing_cars_ids[idx]
            elif lane == veh_lane_no + 1:
                rf = trailing_cars_ids[idx]
            elif lane == veh_lane_no - 1:
                lf = trailing_cars_ids[idx]
                
        for idx, lane in enumerate(leader_cars_lanes):
            if lane == veh_lane_no:
                ml = leader_cars_ids[idx]
            elif lane == veh_lane_no + 1:
                rl = leader_cars_ids[idx]
            elif lane == veh_lane_no - 1:
                ll = leader_cars_ids[idx]
                
        return ll, lf, ml, mf, rl, rf
    
    
      
    '''
        One of the main functions for traffic simulation. It controls the longitudinal accelerations of vehicles.
        For reference, please check the paper itself.
        
        Inputs: v_current, v_desired, d_v, d_s
            v_current : current speed of the vehicle
            v_desired : desired speed of the vehicle
            d_v       : Speed diffrence with leading vehicle, !!! v_current - v_leading !!!
            d_s       : Gap with leading vehicle, !!! position_leading - position_current !!!
        Outputs: v_dot, x_dot, v_dot_unlimited
            v_dot           : Reference Acceleration, that value used for lane change safety check  
    '''
    def IDM(self, v_current, v_desired, d_v, d_s):
        delta = 4  # Acceleration exponent
        a = 0.7  # Maximum acceleration     m/s^2 previous was 0.7
        b = 1.7  # Comfortable Deceleration m/s^2
        th = 1.6  # 2.0 #time headway=1.5 sec
        s0 = 2  # minimum gap =2.0 meters
        s_star = s0 + v_current * th + np.multiply(v_current, d_v) / (2 * np.sqrt(a * b))                  # !!!!!!
        v_dot = a * (1 - np.power((np.divide(v_current, v_desired)), delta) - np.power((np.divide(s_star, d_s + 0.01)), 2))
        v_dot_unlimited = np.copy(v_dot)
        x_dot = v_current
        v_dot[v_dot < -20] = -20  # Lower bound for acceleration, -20 m/s^2
        return v_dot, x_dot, v_dot_unlimited
    
        ''' 
        delta = 4  # Acceleration exponent
        a = 0.7  # Maximum acceleration     m/s^2 previous was 0.7
        b = 1.7  # Comfortable Deceleration m/s^2
        th = 1.6  # 2.0 #time headway=1.5 sec
        s0 = 2  # minimum gap =2.0 meters
        s_star = s0 + v_current * th + np.multiply(v_current, d_v) / (2 * np.sqrt(a * b))             #TODO: !!!!!!
        v_dot = a * (1 - np.power((np.divide(v_current, v_desired)), delta) - np.power((np.divide(s_star, d_s + 0.01)), 2))
        v_dot_unlimited = np.copy(v_dot)
        v_dot[v_dot < -20] = -20  # Lower bound for acceleration, -20 m/s^2
       
        x_dot = v_current
        v_dot[v_dot < -20] = -20  # Lower bound for acceleration, -20 m/s^2
        return v_dot
        '''
        
    
    # Two-Point Visual Control Model of Steering
    # For reference, please check the paper itself.
    # Perform lane change in continuous space with using controller
    # psi -> heading angle.
    def lane_change(self, x_pos_current, y_pos_current, psi_current, v_current, target_lane_id):
        
        pid = PID()
        dynModel = DynModel()
        
        dify = target_lane_id - y_pos_current
        # two points are seleceted within 5 meters and 100 meters, then angles are calculated and fed to PID
        near_error = np.subtract(np.arctan2(dify, 5), psi_current)
        far_error = np.subtract(np.arctan2(dify, 100), psi_current)
        pid_out = pid.update(near_error, far_error, self._dt)
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
        
    def check_safety_criterion(self):
        if self._mode._rule_mode == 0:
            return False;
        else if self._mode._rule_mode ==1:
            pass
        else:
            pass
    
    def check_incentive_criterion(self):
       if self._mode._rule_mode == 0:
            return False;
        else if self._mode._rule_mode ==1:
            pass
        else:
            pass
        
    
    '''
    Algorithm: 
        Calculate the gains of all possible lane change movements.
        According to gains decide where to go.
    '''        
    def MOBIL(self, coordinates, velocities, accelerations, desired_velocities):
                decision
                

        return decision
       
    
    def control(self, veh_coordinates, velocities, desired_v,
                    delta_v, delta_dist, is_lane_changing):
        
        accelerations,_,_ = self.IDM(self._velocities, self._desired_v,
                                              self._delta_v, self._delta_dist)
        
        # Check if the traffic rule enables lane changing.
        if self._mode._rule_mode !=0:
            for vehcl in range(self._dynamics._num_veh):
                if(!is_lane_changing[vehcl])
                    self._decisions[vehcl] = self.MOBIL(veh_coordinates,
                                           accelerations, velocities, desired_v)
                        
        # Execute the lane changing.
       
       
       
       
       
       
       