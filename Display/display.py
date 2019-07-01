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
    # TODO: make the line_height independent!
    # TODO: explain the relationship between gamePlay and display (ex: _window_width)
    # TODO: ask spaceHeight and spaceWidth
    def __init__(self ,background_color = (150, 150, 150) ,text_color = (255, 255, 255) ,game ,line_height = 10 ,line_width = 1):
        
        self._background_color = background_color
        self._text_color = text_color
        
        self._game = game
        self._veh_props = self._game._veh_props
        
        self._line_height = line_height
        self._line_width = line_width
        
        self._window_width = self._game._window_width
        self._window_height = self._game._window_height
        
        self._width_of_lane = ( self._window_height // self._game._dynamics._num_lane )
        
        # TODO: check the necessity of the parameters below!
        self._line_image, self._emergency_line_image = import_lines()
        self._player_rect, self._player_image, self._image_cars = import_images(7)
        
        self._car_images, self._car_rects = create_random_images(self._image_cars)
    
    # The method imports images from the /Image folder.        
    # PyGame related function.
    def import_images(self, num_images):
        
        image_cars = []
    
        for im_i in range(num_images):    
            file = os.path.join(('Images') , 'Car' + str(im_i) + '.png')            
            image_cars.append(pygame.image.load(file).convert())
            image_cars[im_i] = pygame.transform.scale(image_cars[im_i],
                      (self._veh_props._width, self._veh_props._height))
            
        player_rect = image_cars[0].get_rect()
        
        return player_rect, image_cars
    
    #Pygame related function.
    def import_lines(self):
        file = os.path.join(('Images'), 'white.png')
        line_image = (pygame.image.load(file).convert())
        line_image = pygame.transform.scale(line_image, (self._line_height, self._line_width))
        
        file = os.path.join(('Images'), 'white.png')
        emergency_line_image = (pygame.image.load(file).convert())
        emergency_line_image = pygame.transform.scale(emergency_line_image, (self._window_width, self._line_width))

        return line_image, emergency_line_image
    
    # PyGame related function.
    def create_random_images(self, image_cars):

        result_car_images = []
        
        for car in range(self._game._dynamics._num_veh):
            new_image = image_cars[np.random.randint(0, len(image_cars))]
            #        time.sleep(0.1)
            result_car_images.append(new_image)
    
        result_car_rects = []
        for car in range(self._game._dynamics._num_veh):
            new_rect = pygame.Rect(0, 0, self._veh_props._width, self._veh_props._height)
            result_car_rects.append(new_rect)

        return result_car_images, result_car_rects
    
    
    
    # TODO: understand what's going on here!
    # PyGame related function.
    def env_init(self, states):
        
        line_rec_samples = []
        emergency_line_rec_samples = []
        
        for id_of_lane in range(self._game._dynamics._num_lane - 1):
            for coordinates_of_rect in range(self._window_width // (self._line_height * 2)):
                line_x_coord = coordinates_of_rect * self._line_height * 2
                line_y_coord = (id_of_lane + 1) * self._width_of_lane
                new_line_rec = pygame.Rect(line_x_coord, line_y_coord, 0, 0)
                line_rec_samples.append(new_line_rec)

        for id_of_lane in range(self._game_dynamics._num_lane - 1):
            line_y_coord = id_of_lane * self._game._dynamics._num_lane * self._width_of_lane
            new_line_rec = pygame.Rect(0, id_of_lane * (line_y_coord - 10) + 5, 0, 0)
            emergency_line_rec_samples.append(new_line_rec)

        # set up pygame, the window, and the mouse cursor
        os.environ['SDL_VIDEO_CENTERED'] = '1'
        pygame.init()
        main_clock = pygame.time.Clock()
        window_surface = pygame.display.set_mode((self._window_width, self._window_height))
        pygame.display.set_caption('ITSC2019')
        pygame.mouse.set_visible(True)

        window_surface.fill(self._background_color)

        for idx_of_lane in range(0, len(line_rec_samples)):
            window_surface.blit(self.self._line_image, line_rec_samples[idx_of_lane])
        for idx_of_lane in range(0, len(emergency_line_rec_samples)):
            window_surface.blit(self._emergency_line_image, emergency_line_rec_samples[idx_of_lane])
        
        half_lane = (self._width_of_lane // 2 )
        
        player_rect.center = (states[self.ego_id, 1] * 10, half_lane + 2 * half_lane * (states[self.ego_id, 0]))

        window_surface.blit(player_image, player_rect)

        font = pygame.font.SysFont(None, 20)

        for car in range(self._game._dynamics._num_veh):
            if car == self.ego_id:
                continue
            self._car_rects[car].center = (states[car, 1] * 10, half_lane + 2 * half_lane * (states[car, 0]))
            window_surface.blit(self._car_images[car], self._cars_rects[car])
            # self.draw_text(str(car)+','+str(states[car, 0])+','+str(round(states[car, 1], 2)), font, windowsurface,
            #          (states[car, 1])*10-10, half_lane + 2*half_lane*(states[car, 0])-10)
        pygame.display.update()

        return window_surface, main_clock, line_rec_samples, emergency_line_rec_samples
    
    
    # TODO: You are right here. Check the below two methods.
    # PyGame related function.
    def env_update(self, states, line_rec_samples, emergency_line_rec_samples, speed):
        self.WindowSurface.fill(BackgroundColor)

        shift = states[self.ego_veh_id, 1] - WindowWidth / 20
        for idx_of_lane in range(0, len(line_rec_samples)):
            line_rec_samples[idx_of_lane].centerx = (line_rec_samples[idx_of_lane].centerx - shift) % WindowWidth
            self.WindowSurface.blit(self.LineImage, line_rec_samples[idx_of_lane])
        for idx_of_lane in range(0, len(emergency_line_rec_samples)):
            self.WindowSurface.blit(self.EmergencyLaneImage, emergency_line_rec_samples[idx_of_lane])

        PlayerRect.center = (
        (states[self.ego_veh_id, 1] - shift) * 10, half_lane + 2 * half_lane * (states[self.ego_veh_id, 0]))

        self.WindowSurface.blit(PlayerImage, PlayerRect)

        font = pygame.font.SysFont(None, 20)
        for car in range(NoOfCars):
            if car == self.ego_veh_id:
                continue
            self.CarsRecVec[car].center = ((states[car, 1] - shift) * 10, half_lane + 2 * half_lane * (states[car, 0]))
            self.WindowSurface.blit(self.CarsImageVec[car], self.CarsRecVec[car])
            self.draw_text(str(speed[car]), font, self.WindowSurface,
                           (states[car, 1] - shift) * 10 - 30, half_lane + 2 * half_lane * (states[car, 0]) - 5)
        self.draw_text(str(speed[self.ego_veh_id]), font, self.WindowSurface,
                       (states[self.ego_veh_id, 1] - shift) * 10 - 30,
                       half_lane + 2 * half_lane * (states[self.ego_veh_id, 0]) - 5)
        pygame.display.flip()

    # PyGame related function.
    def draw_text(self, text, font, surface, x, y):
        textobj = font.render(text, 1, TextColor)
        textrect = textobj.get_rect()
        textrect.topleft = (x, y)
        surface.blit(textobj, textrect)
        

if __name__ == "__main__": 
    mydisplay = display(background_color = (150, 150, 150), text_color = (255, 255, 255))
    mydisplay.import_images(num_images = 7)
    
    
    