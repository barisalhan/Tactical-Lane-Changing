# -*- coding: utf-8 -*-
"""
Created on 06/26/2019

@author: Baris ALHAN
"""

from Game.gamePlay import gamePlay


# TODO: make the step constant.
# TODO: write mobil movement algorithm.
# TODO: create the right instance of a class at a right place.
# TODO: exlain the general algorithm.
def main():
    
    MAX_STEP = 100000

    game = gamePlay()

    for i in range(MAX_STEP):
        action = game.wait_for_player_to_press_key()
        game.step(action)


if __name__ == "__main__":
    main()
