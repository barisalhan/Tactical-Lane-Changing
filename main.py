# -*- coding: utf-8 -*-
"""
Created on 06/26/2019

@author: Baris ALHAN
"""

import os
import time
import random

import Game
import Display
import Vehicle

from pygame.locals import *

def main():
    #TODO: mobil movement algorithm.
    #TODO: create the right instance of a class at a right place.
    print('inside the main')
    
    # The below constructors are created with default parameters,
    # to read the parameters of a class, go to the related class.
    veh_props = Vehicle.VehicleDynamics.vehiclePhysicalProperties()
    veh_model = Vehicle.VehicleControlModel.dynModel()
    dynamics = Game.gameDynamics()
    mode = Game.gameMode()
    # TODO: check whether display is necessary to send the game as a parameter.
    game = Game.gamePlay(mode = mode, dynamics = dynamics, display = display,
                         veh_props = veh_props, veh_model = veh_model)
    display = Display.display(game = game)
    game.play()

if __name__ == "__main__": 
    main()