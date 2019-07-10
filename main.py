# -*- coding: utf-8 -*-
"""
Created on 06/26/2019

@author: Baris ALHAN
"""

from Vehicle.VehicleDynamics.vehiclePhysicalProperties import vehiclePhysicalProperties
from Vehicle.VehicleControlModel.dynModel import dynModel
from Vehicle.VehicleControl.vehicleAIController import vehicleAIController

from Game.gameDynamics import gameDynamics
from Game.gameMode import gameMode
from Game.gamePlay import gamePlay

from Display.display import display


def main():
    # TODO: make the step constant.
    # TODO: write mobil movement algorithm.
    # TODO: create the right instance of a class at a right place.
    print('inside the main')
    
    # TODO: exlain the general algorithm. 
    
    # The below constructors are created with default parameters,
    # to read about the parameters of a class, go to the related class.
    veh_props = vehiclePhysicalProperties()
    veh_model = dynModel()
    dynamics = gameDynamics()
    mode = gameMode()
    AIController = vehicleAIController()
    # TODO: check display-game relationship.
    game = gamePlay(mode = mode, dynamics = dynamics, 
                         veh_props = veh_props, veh_model = veh_model,
                                 AIController = AIController)
    print('hahaha')
    displayGame = display(game = game)
    
    
    #####################################################
    ###                   Game Play                   ###
    #####################################################
    MAX_STEP = 100000
    
    displayGame.env_init()
   
    for i in range(MAX_STEP):
        action = game.wait_for_player_to_press_key()
        
        
    game.step(action)
    
    
    
    
    
    
    #####################################################
    #####################################################
    
    

if __name__ == "__main__": 
    main()