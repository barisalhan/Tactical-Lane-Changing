# -*- coding: utf-8 -*-
"""
Created on 06/26/2019

@author: Baris ALHAN
"""

from Vehicle.VehicleDynamics.vehiclePhysicalProperties import vehiclePhysicalProperties
from Vehicle.VehicleControlModel.dynModel import dynModel

from Game.gameDynamics import gameDynamics
from Game.gameMode import gameMode
from Game.gamePlay import gamePlay

from Display.display import display


def main():
    #TODO: write mobil movement algorithm.
    #TODO: create the right instance of a class at a right place.
    print('inside the main')
    
    # TODO: exlain the general algorithm. 
    
    # The below constructors are created with default parameters,
    # to read about the parameters of a class, go to the related class.
    veh_props = vehiclePhysicalProperties()
    veh_model = dynModel()
    dynamics = gameDynamics()
    mode = gameMode()
    # TODO: check display-game relationship.
    game = gamePlay(mode = mode, dynamics = dynamics, 
                         veh_props = veh_props, veh_model = veh_model)
    print('hahaha')
    displayGame = display(game = game)
    
    
    #####################################################
    ###                   Game Play                   ###
    #####################################################

    displayGame.env_init()
    
    
    
    
    
    
    #####################################################
    #####################################################
    
    

if __name__ == "__main__": 
    main()