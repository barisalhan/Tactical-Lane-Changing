# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 10:32:23 2019

@author: Baris ALHAN
"""
import numpy as np
class reinforcementLearning:
    
    def __init__(self, time_of_collision_low, time_of_collision_high):
        self._time_of_collision_low = time_of_collision_low
        self._time_of_collision_high = time_of_collision_high
    
    
    
    # RL related function.
    def get_input_states(self, states, V_vehicles, t):
        # s_max = max(abs(states[:, 1] - states[self.ego_veh_id, 1]))
        # s_max = v_ego_max * t
        # v_max = max(V_vehicles)
        s_max = GOAL_DISTANCE # distance_long/2+ safety margin will be 50
        v_max = v_d[self.ego_veh_id]
        input_vector = np.zeros((3*NoOfCars, 1))
        input_vector[0] = np.random.normal(V_vehicles[self.ego_veh_id] / v_max, 0.1) # our speed unceert
    
        if states[self.ego_veh_id, 0] == 0:
            input_vector[1] = 0
        else:
            input_vector[1] = 1
        if states[self.ego_veh_id, 0] == (NoOfLanes-1):
            input_vector[2] = 0
        else:
            input_vector[2] = 1
    
        for idx in range(0, self.ego_veh_id):
            input_vector[3 * (idx + 1)] = np.random.normal(((states[idx, 1] - states[self.ego_veh_id, 1]) / s_max), 0.3) # uncert for distance
            input_vector[3 * (idx + 1) + 1] = np.random.normal(V_vehicles[idx] / v_max, 0.2) # uncert for speed
            input_vector[3 * (idx + 1) + 2] = (states[idx, 0] - states[self.ego_veh_id, 0])/2
    
        for idx in range(self.ego_veh_id+1, NoOfCars):
            input_vector[3 * (idx )] = np.random.normal((states[idx, 1] - states[self.ego_veh_id, 1]) / s_max, 0.3) # uncert for distance
            input_vector[3 * (idx) + 1] = np.random.normal(V_vehicles[idx] / v_max, 0.2) # uncert for speed
            input_vector[3 * (idx) + 2] = (states[idx, 0] - states[self.ego_veh_id, 0])/2
            
        return input_vector    
        
    
    
    # TODO: HERE!
    # TODO: understand what's going on here!
    # TODO: explain the general behavior of the method.
    # this is for RL agents
    def step(self, action):
            
        isDone = False
        
        Near_Collision = False
        Hard_Collision = False
        
        # TODO: ??
        rew = np.array([0.])
        
        #
        decisions = np.zeros((self._dynamics._num_veh, 1))
        #
        gains = np.zeros((self._dynamics._num_veh, 1))

        # TODO: YOU ARE HERE!
        V_dot, X_dot, _ = self.idm(self._current_velocities, self._desired_v,
                                   self._delta_v, self._delta_dist)
        

        # If traffic rule is USA or UK.
        if self._mode._rule_mode == 1 or self._mode._rule_mode == 2:
            if self._time > 0:
                for id in range(0, self._dynamics._num_veh):
                    ll_id, lf_id, ml_id, mf_id, rl_id, rf_id = self.check_car_id(self._veh_coordinates, id)
                    decisions[id], gains[id] = self.check_safety(id, self._veh_coordinates, V_dot, self._current_velocities,
                             self._delta_v, ll_id, lf_id, ml_id, mf_id, rl_id, rf_id)

            for id in range(0, self._dynamics._num_veh):
                if decisions[id] != 0 and id == np.argmax(gains):
                    self.target_lane[id] = self._veh_coordinates[id, 0] + decisions[id]
                    if self.target_lane[id] not in range(0, self._dynamics._num_):
                        self.target_lane[id] = self.States[idx, 0]
            self.target_lane[self.ego_veh_id] = self.States[self.ego_veh_id, 0]
            
        if action == 2:
            rew += -1
            action = decisions[self.ego_veh_id]
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
    