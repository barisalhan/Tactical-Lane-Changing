# -*- coding: utf-8 -*-
"""
Created on Tue Jul  2 10:38:11 2019

@author: Baris ALHAN
"""

class vehicleAIController:
    
    
    
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
    def check_safety(self, vehicle_id, states, x_ddot, v_current, v_desired, ll, lf, ml, mf, rl, rf):
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