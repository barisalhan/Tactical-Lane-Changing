# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 10:34:11 2019

@author: Baris ALHAN
"""
import Vehicle.VehicleDynamics.vehiclePhysicalProperties as vehicle
import Game.gameDynamics as dynamics

import pygame
import os

class display:
    
    def __init__(self, background_color = (150, 150, 150), text_color = (255, 255, 255)):
        
        self._background_color = background_color
        self._text_color = text_color
        
    
    
    # PyGame related function.
    def import_images(self, num_images):
        
        image_cars = []
        
        vehicleProperties = vehicle.VehiclePhysicalProperties()
        
        for im_i in range(num_images):
            
            file = os.path.join(('Images') , 'Car' + str(im_i) + '.png')
            
            image_cars.append(pygame.image.load(file).convert())
            image_cars[im_i] = pygame.transform.scale(image_cars[im_i],
                      (vehicleProperties._width,vehicleProperties._height))
            
        player_rect = image_cars[0].get_rect()
        
        return player_rect, image_cars
    
    # TODO: You are right here!
    # PyGame related function.
    def create_random_images(self, carsimage):

        carsimagevec = []
        for car in range(NoOfCars):
            newimage = carsimage[np.random.randint(0, len(carsimage))]
            #        time.sleep(0.1)
            carsimagevec.append(newimage)

        lineimage = pygame.image.load('image/white.png')
        lineimage = pygame.transform.scale(lineimage, (LineHeight, LineWidth))
        emergency_lineimage = pygame.image.load('image/white.png')
        emergency_lineimage = pygame.transform.scale(emergency_lineimage, (WindowWidth, LineWidth))

        carsrecvec = []
        for car in range(NoOfCars):
            newrec = pygame.Rect(0, 0, CarWidth, CarHeight)
            carsrecvec.append(newrec)

        return carsimagevec, carsrecvec, lineimage, emergency_lineimage
    



if __name__ == "__main__": 
    mydisplay = display(background_color = (150, 150, 150), text_color = (255, 255, 255))
    mydisplay.import_images(num_images = 7)
    
    
    