# -*- coding: utf-8 -*-
"""
Created on Tue Jul  2 10:38:11 2019

@author: Baris ALHAN
"""
import numpy as np
from Vehicle.VehicleControlModel.PID import PID
from Vehicle.VehicleControlModel.dynModel import dynModel as DynModel

class vehicleAIController: 
    
    def __init__(self,dt):
        self._dt = dt
    
    # TODO: implement the algorithm again!
    ## Function that returns the IDs of surrounding vehicle for each vehicle
    # mf, middle follower
    # rf, right follower 
    # lf, left follower
    # ml, middle leader
    # rl, right leader
    # ll, left leader 
    def get_surrounding_vehs(self, states, vehicle_id):
        # each vehicle has static unique id
        # algorithm:
        # araclarin pozisyonun gore sirala
        # aracin hangi lane'de oldugunu bul
        # en yakin araclari tespit et.
        mf, rf, lf = None, None, None
        ml, rl, ll = None, None, None
        
        sorted_ids = list(np.argsort(states[:, 1]))
        
        veh_lane_no = states[vehicle_id, 0]
        
        veh_id_sorted = sorted_ids.index(vehicle_id)
        
        trailing_cars_ids = np.argsort(states[:, 1])[0:veh_id_sorted]
        trailing_cars_lanes = states[sorted_ids[0:veh_id_sorted], 0]
        
        leader_cars_ids = np.flip(np.argsort(states[:, 1])[(veh_id_sorted + 1):])
        leader_cars_lanes = np.flip(states[sorted_ids[(veh_id_sorted + 1):], 0])
        
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
        s_star = s0 + v_current * th + np.multiply(v_current, d_v) / (2 * np.sqrt(a * b))             #TODO: !!!!!!
        v_dot = a * (1 - np.power((np.divide(v_current, v_desired)), delta) - np.power((np.divide(s_star, d_s + 0.01)), 2))
        v_dot[v_dot < -20] = -20  # Lower bound for acceleration, -20 m/s^2
        
        return v_dot
    
    
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

    
    
    ## Function that returns safety decision for lane change (MOBIL)
    ## Inputs: vehicle_id, states, x_ddot, v_current, v_desired, ll, lf, ml, mf, rl, rf
    # vehicle_id             : current vehicle ID 
    # states                 : positions of other vehicles
    # x_ddot                 : acceleration of current vehicle
    # v_current              : speed of current vehicle
    # v_desired              : desired speed of current vehicle
    # ll, lf, ml, mf, rl, rf : surrounding vehicle IDs for current vehicle
    ## Outputs: change_decision, safety_gain
    # change_decision : lane change decision, -1:left, 0:stay, 1:right
    # safety_gain : gain of decision   
    #### Dummy Variables
    # a_n_l, a_n_r, a_o     # Accelerations of left, right, middle followers
    # a_e_hat_l, a_e_hat_r  # Future accelerations of current vehicle for left and right lane change which calculated vy using IDM for one step
    # a_n_l_hat, a_n_r_hat  # Future accelerations of left and right follower vehicle for left and right lane change which calculated vy using IDM for one step
    # a_o_hat               # Future accelerations of middel follower vehicle for left and right lane change which calculated vy using IDM for one step
    def MOBIL(self, vehicle_id, states, x_ddot, v_current, v_desired, ll, lf, ml, mf, rl, rf):
        a_n_l, a_n_r, a_o = None, None, None   # Accelerations of left, right, middle followers
        a_e_hat_l, a_e_hat_r, a_n_l_hat, a_n_r_hat, a_o_hat = None, None, None, None, None # Future accelerations of left, right, middle followers

        min_lane_id = min(states[:, 0])
        max_lane_id = max(states[:, 0])

        a_e = x_ddot[vehicle_id]
        if lf is not None:
            a_n_l = x_ddot[lf]
        if lf is None:
            if states[vehicle_id, 0] == min_lane_id:
                a_n_l = [1000]
            else:
                a_n_l = [0]
        if mf is not None:
            a_o = x_ddot[mf]
        if mf is None:
            a_o = [0]
        if rf is not None:
            a_n_r = x_ddot[rf]
        if rf is None:
            if states[vehicle_id, 0] == max_lane_id:
                a_n_r = [1000]
            else:
                a_n_r = [0]

        if ll is not None:
            d_v_l = v_current[vehicle_id] - v_current[ll]
            d_s_l = states[ll, 1] - states[vehicle_id, 1] - 0
            if d_s_l < 5:
                d_s_l = 0
            _, x_dot_l_hat, a_e_hat_l = self.idm(v_current[vehicle_id], v_desired[vehicle_id], d_v_l, d_s_l)
        elif ll is None:
            if states[vehicle_id, 0] == min_lane_id:
                a_e_hat_l, x_dot_l_hat = [-1000], [0]
            else:
                d_v_l = 0
                d_s_l = 10 ** 5
                _, x_dot_l_hat, a_e_hat_l = self.idm(v_current[vehicle_id], v_desired[vehicle_id], d_v_l, d_s_l)

        if rl is not None:
            d_v_r = v_current[vehicle_id] - v_current[rl]
            d_s_r = states[rl, 1] - states[vehicle_id, 1] - 0
            if d_s_r < 5:
                d_s_r = 0
            _, x_dot_r_hat, a_e_hat_r = self.idm(v_current[vehicle_id], v_desired[vehicle_id], d_v_r, d_s_r)

        if rl is None:
            if states[vehicle_id, 0] == max_lane_id:
                a_e_hat_r, x_dot_r_hat = [-1000], [0]
            else:
                d_v_r = 0
                d_s_r = 10 ** 5
                _, x_dot_r_hat, a_e_hat_r = self.idm(v_current[vehicle_id], v_desired[vehicle_id], d_v_r, d_s_r)

        if lf is not None:
            d_v_n_l = v_current[lf] - v_current[vehicle_id]
            d_s_n_l = states[vehicle_id, 1] - states[lf, 1] - 0
            if d_s_n_l < 5:
                d_s_n_l = 0
            _, x_dot_n_l_hat, a_n_l_hat = self.idm(v_current[lf], v_desired[lf], d_v_n_l, d_s_n_l)
        if lf is None:
            if states[vehicle_id, 0] == min_lane_id:
                a_n_l_hat, x_dot_n_l_hat = [-1000], [0]
            else:
                a_n_l_hat, x_dot_n_l_hat = [0], [0]

        if rf is not None:
            d_v_n_r = v_current[rf] - v_current[vehicle_id]
            d_s_n_r = states[vehicle_id, 1] - states[rf, 1] - 0
            if d_s_n_r < 5:
                d_s_n_r = 0
            _, x_dot_n_r_hat, a_n_r_hat = self.idm(v_current[rf], v_desired[rf], d_v_n_r, d_s_n_r)

        if rf is None:
            if states[vehicle_id, 0] == max_lane_id:
                a_n_r_hat, x_dot_n_r_hat = [-1000], [0]
            else:
                a_n_r_hat, x_dot_n_r_hat = [0], [0]

        if mf is not None and ml is not None:
            d_v_0 = v_current[mf] - v_current[ml]
            d_s_0 = states[ml, 1] - states[mf, 1] - 0
            if d_s_0 < 5:
                d_s_0 = 0
            _, x_dot_n_r_hat, a_o_hat = self.idm(v_current[mf], v_desired[mf], d_v_0, d_s_0)

        if mf is not None and ml is None:
            d_v_0 = 0
            d_s_0 = 10 ** 5
            _, x_dot_n_r_hat, a_o_hat = self.idm(v_current[mf], v_desired[mf], d_v_0, d_s_0)

        if mf is None:
            a_o_hat, x_dot_n_r_hat = [0], [0]

        p = 0
        q = 0
        # these are mobil equations, P, Q => politnesss thresholds!!
        changing_threshold_a_th = 0.00  # 0.1
        b_safe = 4
        safety_gain_left = (a_e_hat_l[0] - a_e[0]) + p * (a_n_l_hat[0] - a_n_l[0]) + q * (a_o_hat[0] - a_o[0])
        safety_gain_right = (a_e_hat_r[0] - a_e[0]) + p * (a_n_r_hat[0] - a_n_r[0]) + q * (a_o_hat[0] - a_o[0])

        safety_left, safety_right = False, False
        # if USA, choose more safe lane change in terms of left and right
        if a_n_l_hat[0] > -b_safe:
            if safety_gain_left > changing_threshold_a_th:
                safety_left = True

        if a_n_r_hat[0] > -b_safe:
            if safety_gain_right > changing_threshold_a_th:
                safety_right = True

        safety_gain = 0
        change_decision = 0  # Stay current lane

        if traffic_rules == 'EU':
            if safety_left is True and safety_right is True and ml is not None and mf is not None:
                change_decision = -1  # Shift left lane
                safety_gain = safety_gain_left
            elif safety_left is True and safety_right is True and ml is None and mf is not None:
                change_decision = 1  # Shift right lane
                safety_gain = safety_gain_right
            elif safety_left is True and safety_right is True and ml is not None and mf is None:
                change_decision = -1  # Shift left lane
                safety_gain = safety_gain_left
            elif safety_left is False and safety_right is True and ml is None and mf is not None:
                change_decision = 1  # Shift right lane
                safety_gain = safety_gain_right
            elif safety_left is True and safety_right is False and ml is not None and mf is not None:
                change_decision = -1  # Shift left lane
                safety_gain = safety_gain_left
            elif safety_left is True and safety_right is False and ml is not None and mf is None:
                change_decision = -1  # Shift left lane
                safety_gain = safety_gain_left
            else:
                change_decision = 0  # Stay current lane

        elif traffic_rules == 'USA':
            if safety_left is True and safety_right is True and ml is not None and mf is not None:
                if safety_gain_left < safety_gain_right:
                    change_decision = 1  # Shift right lane
                    safety_gain = safety_gain_right
                else:
                    change_decision = -1  # Shift left lane
                    safety_gain = safety_gain_left
            elif safety_left is True and safety_right is True and ml is None and mf is not None:
                if safety_gain_left < safety_gain_right:
                    change_decision = 1  # Shift right lane
                    safety_gain = safety_gain_right
                else:
                    change_decision = -1  # Shift left lane
                    safety_gain = safety_gain_left
            elif safety_left is True and safety_right is True and ml is not None and mf is None:
                if safety_gain_left < safety_gain_right:
                    change_decision = 1  # Shift right lane
                    safety_gain = safety_gain_right
                else:
                    change_decision = -1  # Shift left lane
                    safety_gain = safety_gain_left
            elif safety_left is False and safety_right is True and ml is not None and mf is not None:
                change_decision = 1  # Shift right lane
                safety_gain = safety_gain_right
            elif safety_left is False and safety_right is True and ml is None and mf is not None:
                change_decision = 1  # Shift right lane
                safety_gain = safety_gain_right
            elif safety_left is False and safety_right is True and ml is not None and mf is None:
                change_decision = 1  # Shift right lane
                safety_gain = safety_gain_right
            elif safety_left is True and safety_right is False and ml is not None and mf is not None:
                change_decision = -1  # Shift left lane
                safety_gain = safety_gain_left
            elif safety_left is True and safety_right is False and ml is not None and mf is None:
                change_decision = -1  # Shift left lane
                safety_gain = safety_gain_left
            else:
                change_decision = 0  # Stay current lane
        else:
            safety_gain = 0
            change_decision = 0  # Stay current lane

        return change_decision, safety_gain