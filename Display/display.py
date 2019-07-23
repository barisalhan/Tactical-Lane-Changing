# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 10:34:11 2019

@author: Baris ALHAN
"""

import pygame
from pygame.locals import *

import os
import numpy as np


class display:
    
    # TODO: adjust code to enable no-rendering mode.
    def __init__(self, game):
             
        self._game = game
        
        #######################################################################
        #####                   BACKGROUND PROPERTIES                     #####
        #######################################################################
        # I prefer to not parametrize these 4 variables, because
        # in that case the overall software would be overly-generic.
        self._background_color = (150,150,150)
        self._text_color  = (255, 255, 255)
        
        # The height and width of the road lines in the game.
        self._line_height = 10
        self._line_width = 1
        
        self._window_width = int(self._game._dynamics._max_veh_inlane * 
                               (self._game._veh_props._width + 2 * self._game._veh_props._height))
        self._window_height = \
            int((self._game._veh_props._height +  2 * (self._game._veh_props._height // 2.5)) * self._game._dynamics._num_lane )
        
        # The width of the lane in the road.
        self._width_of_lane =  (self._window_height // self._game._dynamics._num_lane)
        #######################################################################
        #######################################################################     
        

        #######################################################################
        #####                INITIALIZATION OF THE PYGAME                 #####
        #######################################################################
        os.environ['SDL_VIDEO_CENTERED'] = '1'
        # Set up the pygame
        pygame.init()
        
        self._window_surface = pygame.display.set_mode((self._window_width,
                                                        self._window_height))
        
        self._main_clock = pygame.time.Clock()
        
        pygame.display.set_caption('ITSC2019')
        # Set the mouse cursor
        pygame.mouse.set_visible(True)
        #######################################################################
        #######################################################################
               
        
        #######################################################################
        #####                        THE IMAGES                           #####
        #######################################################################
        # _images_veh holds the all possible vehicle images.
        self._images_veh = self.import_images(7)
        # the images of road lines used in displaying
        self._line_image, self._emergency_line_image = self.import_line_images()
        # _vehcls_images holds the image of each vehicle in the game.
        self._vehcls_image = self.assign_images_to_vehicles(self._images_veh)
        # _vehcls_rect holds the rectangle of each vehicle in the game.
        self._vehcls_rect = self.get_vehcls_rect()
        # _lines_rect holds the rectangle of each road line in the map.
        self._lines_rect, self._emergency_lines_rect = self.get_lines_rect()
        #######################################################################
        #######################################################################
        
    ###########################################################################
    
    
    
    # The method imports images from the Display/Image folder.
    # Possible future erroreneous change:
    # If you change the file path, make sure you really understood that it is
    # the relative path with respect to main.py, not display.py!        
    # PyGame related function.
    def import_images(self, num_images):
        
        images_veh = []
    
        for im_i in range(num_images):    
            file = os.path.join('Display', 'Images' , 'Car' + str(im_i) + '.png')
            images_veh.append(pygame.image.load(file).convert())
            images_veh[im_i] = pygame.transform.scale(images_veh[im_i],
                      (self._game._veh_props._width, self._game._veh_props._height))
        
        return images_veh
    
    #PyGame related function.
    def import_line_images(self):
        
        file2 = os.path.join('Display', 'Images' , 'white.png')
        line_image = pygame.image.load(file2).convert()
        line_image = pygame.transform.scale(line_image, (self._line_height, self._line_width))
        
        file3 = os.path.join('Display','Images', 'white.png')
        emergency_line_image = (pygame.image.load(file3).convert())
        emergency_line_image = pygame.transform.scale(emergency_line_image, (self._window_width, self._line_width))

        return line_image, emergency_line_image
    
    # PyGame related function.
    def assign_images_to_vehicles(self, image_veh):

        result_vehcls_image = []
        
        for veh in range(self._game._dynamics._num_veh):
            new_image = image_veh[np.random.randint(0, len(image_veh))]
            #        time.sleep(0.1)
            result_vehcls_image.append(new_image)

        return result_vehcls_image
    
    #PyGame related function.
    def get_vehcls_rect(self):
        
        result_vehcls_rect = []
        
        for car in range(self._game._dynamics._num_veh):
            new_rect = pygame.Rect(0, 0, self._game._veh_props._width,
                                   self._game._veh_props._height)
            result_vehcls_rect.append(new_rect)
        
        return result_vehcls_rect
    
    #PyGame related function    
    def get_lines_rect(self):
        # The lists to hold the rectangles of the line images.
        lines_rect = []
        emergency_lines_rect = []
        
        #Determining the position of the road lines.
        for id_of_lane in range(self._game._dynamics._num_lane - 1):
            for coordinates_of_rect in range(self._window_width // (self._line_height * 2)):
                line_x_coord = coordinates_of_rect * self._line_height * 2
                line_y_coord = (id_of_lane + 1) * self._width_of_lane
                new_line_rect = pygame.Rect(line_x_coord, line_y_coord, 0, 0)
                lines_rect.append(new_line_rect)
        
        #Determining the position of the emergency lines.
        for id_of_lane in range(self._game._dynamics._num_lane - 1):
            line_y_coord = id_of_lane * self._game._dynamics._num_lane * self._width_of_lane
            new_line_rect = pygame.Rect(0, id_of_lane * (line_y_coord - 10) + 5, 0, 0)
            emergency_lines_rect.append(new_line_rect)
            
        return lines_rect, emergency_lines_rect


    # This method is the point where the visual environment
    # of the game is first created.
    # PyGame related function.
    def env_init(self):  
        
        self._states = self._game._veh_coordinates
        self._window_surface.fill(self._background_color)
        
        # Drawing lines to the screen
        for line in range(0, len(self._lines_rect)):
            self._window_surface.blit(self._line_image, self._lines_rect[line])
        # Drawing emergency lines to the screen 
        for emergency_line in range(0, len(self._emergency_lines_rect)):
            self._window_surface.blit(self._emergency_line_image, self._emergency_lines_rect[emergency_line])
        
        half_lane = (self._width_of_lane // 2 )
        
        # Drawing vehicles to the screen
        for veh in range(self._game._dynamics._num_veh):
            self._vehcls_rect[veh].center = (self._states[veh, 1] * 10, half_lane + 2 * half_lane * (self._states[veh, 0]))
            self._window_surface.blit(self._vehcls_image[veh], self._vehcls_rect[veh])
            
        pygame.display.update()
        self.env_update()

    # PyGame related function.
    def env_update(self):
        
        self._states = self._game._veh_coordinates
        self._window_surface.fill(self._background_color)

        shift = self._states[self._game._ego_id, 1] - self._window_width / 20
        
        # Shifting the lines and drawing to the screen.
        for line in range(0, len(self._lines_rect)):
            self._lines_rect[line].centerx = (self._lines_rect[line].centerx - shift) % self._window_width
            self._window_surface.blit(self._line_image, self._lines_rect[line])
        # Drawing the emergency lines to the screen        
        for emergency_line in range(0, len(self._emergency_lines_rect)):
            self._window_surface.blit(self._emergency_line_image, self._emergency_lines_rect[emergency_line])
            
        half_lane = (self._width_of_lane // 2 )

        font = pygame.font.SysFont(None, 20)
        
        # Drawing vehicles and speeds to the screen
        for veh in range(self._game._dynamics._num_veh):
            self._vehcls_rect[veh].center = ((self._states[veh, 1] - shift) * 10, half_lane + 2 * half_lane * (self._states[veh, 0]))
            self._window_surface.blit(self._vehcls_image[veh], self._vehcls_rect[veh])

            self.draw_text(str(self._game._velocities[veh]), font, self._window_surface,
                           self._vehcls_rect[veh].centerx - 30,  self._vehcls_rect[veh].centery - 5)
        
        pygame.display.flip()    
        self._main_clock.tick()
        pygame.event.pump()
        
    # PyGame related function.
    def draw_text(self, text, font, surface, x, y):
        text_obj = font.render(text, 1, self._text_color)
        text_rect = text_obj.get_rect()
        text_rect.topleft = (x, y)
        surface.blit(text_obj, text_rect)
        
        