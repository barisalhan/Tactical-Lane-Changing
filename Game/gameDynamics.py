# -*- coding: utf-8 -*-
"""
Created on Fri Jun 28 11:58:37 2019

@author: Baris ALHAN
"""

class gameDynamics:
    '''
        The class for the settings of the more dynamic variables that may be
        desired to be changed in each simulation scenerios independent of the
        game mode.
        
        @Params:
            num_veh:
            num_actions:
            desired_min_speed: The beginning min. velocity for the all vehicles
            desired_max_speed: The beginning max. velocity for the all vehicles
            max_veh_inlane:    
                
    '''
    # TODO: understand what the action is
    def __init__(self, num_veh = 9, num_lane = 3, num_actions = 3, max_veh_inlane = 20, desired_min_v = 18, desired_max_v = 26):
        self._num_veh = num_veh
        self._num_lane = num_lane
        self._num_actions = num_actions
        self._max_veh_inlane = max_veh_inlane
        self._desired_min_v = desired_min_v
        self._desired_max_v = desired_max_v