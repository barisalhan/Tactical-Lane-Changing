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
    # TODO: it may require to parametrize line_height.
    # TODO: explain the relationship between gamePlay and display (ex: _window_width)
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
        
        self._line_height = 10
        self._line_width = 1
        
        self._window_width = self._game._window_width
        self._window_height = self._game._window_height
        
        self._width_of_lane =  (self._window_height // self._game._dynamics._num_lane)
        #######################################################################
        #######################################################################
        
        
        #######################################################################
        #####                INITIALIZATION OF THE PYGAME                 #####
        #######################################################################
        os.environ['SDL_VIDEO_CENTERED'] = '1'
        # set up the pygame
        pygame.init()
        
        self._window_surface = pygame.display.set_mode((self._window_width,
                                                        self._window_height))
        
        pygame.display.set_caption('ITSC2019')
        # set the mouse cursor
        pygame.mouse.set_visible(True)
        #######################################################################
        #######################################################################
             
        
        #######################################################################
        #####                        THE IMAGES                           #####
        #######################################################################
        # _images_veh holds the all possible vehicle images.
        self._images_veh = self.import_images(7)
        # the images of road lines used in displaying
        self._line_image, self._emergency_line_image = self.import_lines()
        # _vehs_images holds the image for each of the vehicle
        self._vehs_image = self.assign_images_to_vehicles(self._images_veh)
        # _vehs_rect holds the rectangle for each of the vehicle
        self._vehs_rect = self.get_vehs_rect()
        #######################################################################
        #######################################################################
        
        self._states = self._game._veh_coordinates        
    
    
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
    def import_lines(self):
        
        file2 = os.path.join('Display', 'Images' , 'white.png')
        line_image = pygame.image.load(file2).convert()
        line_image = pygame.transform.scale(line_image, (self._line_height, self._line_width))
        
        file3 = os.path.join('Display','Images', 'white.png')
        emergency_line_image = (pygame.image.load(file3).convert())
        emergency_line_image = pygame.transform.scale(emergency_line_image, (self._window_width, self._line_width))

        return line_image, emergency_line_image
    
    # PyGame related function.
    def assign_images_to_vehicles(self, image_veh):

        result_vehs_image = []
        
        for veh in range(self._game._dynamics._num_veh):
            new_image = image_veh[np.random.randint(0, len(image_veh))]
            #        time.sleep(0.1)
            result_vehs_image.append(new_image)

        return result_vehs_image
    
    #PyGame related function.
    def get_vehs_rect(self):
        
        result_vehs_rect = []
        
        for car in range(self._game._dynamics._num_veh):
            new_rect = pygame.Rect(0, 0, self._game._veh_props._width,
                                   self._game._veh_props._height)
            result_vehs_rect.append(new_rect)
        
        return result_vehs_rect
    
    
    '''
        This method is the point where the visual environment of the roads are
        first created.
    '''
    # TODO: check window_surface
    # PyGame related function.
    def env_init(self):
        
        
        # The lists to hold the rectangles of the lines
        line_rects = []
        emergency_line_rects = []
        
        #Determining the position of the road lines.
        for id_of_lane in range(self._game._dynamics._num_lane - 1):
            for coordinates_of_rect in range(self._window_width // (self._line_height * 2)):
                line_x_coord = coordinates_of_rect * self._line_height * 2
                line_y_coord = (id_of_lane + 1) * self._width_of_lane
                new_line_rect = pygame.Rect(line_x_coord, line_y_coord, 0, 0)
                line_rects.append(new_line_rect)
        
        #Determining the position of the emergency lines.
        for id_of_lane in range(self._game._dynamics._num_lane - 1):
            line_y_coord = id_of_lane * self._game._dynamics._num_lane * self._width_of_lane
            new_line_rect = pygame.Rect(0, id_of_lane * (line_y_coord - 10) + 5, 0, 0)
            emergency_line_rects.append(new_line_rect)


        main_clock = pygame.time.Clock()
        
        self._window_surface.fill(self._background_color)
         
        ego_id = self._game._ego_id
        ego_image = self._vehs_image[ego_id]
        ego_rect = self._vehs_rect[ego_id]
        
        # Drawing lines to the screen
        for line in range(0, len(line_rects)):
            self._window_surface.blit(self._line_image, line_rects[line])
        # Drawing emergency lines to the screen.    
        for emergency_line in range(0, len(emergency_line_rects)):
            self._window_surface.blit(self._emergency_line_image, emergency_line_rects[emergency_line])
        
        half_lane = (self._width_of_lane // 2 )
        
        ego_rect.center = ( self._states[ego_id, 1] * 10,
                      half_lane + 2 * half_lane * (self._states[ego_id, 0]) )

        self._window_surface.blit(ego_image, ego_rect)

        font = pygame.font.SysFont(None, 20)

        for veh in range(self._game._dynamics._num_veh):
            if veh == ego_id:
                continue
            self._vehs_rect[veh].center = (self._states[veh, 1] * 10, half_lane + 2 * half_lane * (self._states[veh, 0]))
            self._window_surface.blit(self._vehs_image[veh], self._vehs_rect[veh])
            # self.draw_text(str(car)+','+str(states[car, 0])+','+str(round(states[car, 1], 2)), font, windowsurface,
            #          (states[car, 1])*10-10, half_lane + 2*half_lane*(states[car, 0])-10)
        pygame.display.update()
        
        return  main_clock, line_rects, emergency_line_rects
    
    
    # PyGame related function.
    # TODO: get states
    def env_update(self, states, line_rec_samples, emergency_line_rec_samples, speed):
        
        self._window_surface.fill(self._background_color)

        shift = states[self.ego_veh_id, 1] - self._window_width / 20
        for idx_of_lane in range(0, len(line_rec_samples)):
            line_rec_samples[idx_of_lane].centerx = (line_rec_samples[idx_of_lane].centerx - shift) % self._window_width
            self._window_surface.blit(self._line_image, line_rec_samples[idx_of_lane])
        for idx_of_lane in range(0, len(emergency_line_rec_samples)):
            self._window_surface.blit(self._emergency_line_image, emergency_line_rec_samples[idx_of_lane])

        half_lane = (self._width_of_lane // 2 )
        
        PlayerRect.center = (
        (states[self.ego_veh_id, 1] - shift) * 10, half_lane + 2 * half_lane * (states[self.ego_veh_id, 0]))

        self._window_surface.blit(self._player_image, self._player_rect)

        font = pygame.font.SysFont(None, 20)
        for veh in range(self._game._dynamics._num_veh):
            if veh == self.ego_veh_id:
                continue
            self._veh_rects[veh].center = ((states[veh, 1] - shift) * 10, half_lane + 2 * half_lane * (states[veh, 0]))
            self._window_surface.blit(self._veh_images[car], self._veh_rects[car])
            self.draw_text(str(speed[veh]), font, self._window_surface,
                           (states[veh, 1] - shift) * 10 - 30, half_lane + 2 * half_lane * (states[veh, 0]) - 5)
        self.draw_text(str(speed[self.ego_veh_id]), font, self._window_surface,
                       (states[self.ego_veh_id, 1] - shift) * 10 - 30,
                       half_lane + 2 * half_lane * (states[self.ego_veh_id, 0]) - 5)
        pygame.display.flip()
        
        
        
    # PyGame related function.
    def draw_text(self, text, font, surface, x, y):
        text_obj = font.render(text, 1, self._text_color)
        text_rect = text_obj.get_rect()
        text_rect.topleft = (x, y)
        surface.blit(text_obj, text_rect)
        

if __name__ == "__main__": 
    mydisplay = display(background_color = (150, 150, 150), text_color = (255, 255, 255))
    mydisplay.import_images(num_images = 7)
    
    
    