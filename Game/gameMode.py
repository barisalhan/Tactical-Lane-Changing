# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 10:26:13 2019

@author: Baris ALHAN
"""

from enum import Enum    

class gameMode:
    '''
        The class for the settings of the general game mode.
        @Params:
            is_rendering: if false, there will be no display
            rule_mode:
                0 -> No lane Change
                1 -> USA traffic rules
                2 -> UK traffic rules
            behavior_mode: The behavioral models of other actors.
                # TODO: add behavioral models.
            distance_goal:
    '''
    def __init__(self, is_rendering=True, rule_mode = 1, behavior_mode = 0, distance_goal = 1000):
        self._is_rendering = is_rendering
        self._rule_mode = rule_mode
        self._behavior_mode = behavior_mode
        self._distance_goal = distance_goal
        