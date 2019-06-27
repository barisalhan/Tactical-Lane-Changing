# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 10:26:13 2019

@author: Baris ALHAN
"""

class gameMode:
    
    def __init__(self, is_rendering=True, rule_mode, behavior_mode, distance_goal, num_cars, num_actions):
        self._is_rendering = is_rendering
        self._rule_mode = rule_mode
        self._behavior_mode = behavior_mode
        self._distance_goal = distance_goal
        self._num_cars = num_cars
        self._num_actions = num_actions