# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 10:34:11 2019

@author: Baris ALHAN
"""
import Vehicle.VehicleDynamics.vehiclePhysicalProperties as vehicle

import pygame
import os

class display:
    
    def __init__(self, background_color, text_color):
        self._background_color = background_color
        self._text_color = text_color
    
    
    # PyGame related function.
    def import_images(self, num_images):
        
        image_cars = []
        
        vehicleProperties = vehicle.VehiclePhysicalProperties()
        
        for im_i in range(num_images):
            
            file = os.path.join(('Images') , 'Car' + str(im_i) + '.png')
            
            image_cars.append(pygame.image.load(file))
            image_cars[im_i] = pygame.transform.scale(image_cars[im_i],
                      (vehicleProperties._width,vehicleProperties._height))
            
        player_rect = image_cars[0].get_rect()
        
        return player_rect, image_cars
    
    


if __name__ == "__main__": 
    mydisplay = display(background_color = (150, 150, 150), text_color = (255, 255, 255))
    mydisplay.import_images(num_images = 7)
    
    
    