# -*- coding: utf-8 -*-
"""
Created on 06/26/2019

@author: Baris ALHAN
"""

from Game.gamePlay import gamePlay


def main():
    '''
    Handles the connection with the user and the game engine.
    '''
    
    #: int: The length of lifetime of the game in terms of the step number.
    MAX_STEP = 100000
    
    game = gamePlay()
    
    
    for num_step in range(MAX_STEP):
        for vehcl in game._vehicles:
            print("Id:{}. target_lane:{}".format(vehcl._id,vehcl._target_lane))
        action = game.wait_for_player_to_press_key()
        game.step(action)
    


if __name__ == "__main__":
    main()
