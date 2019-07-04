# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 17:19:57 2019

@author: Baris ALHAN
"""
class vehiclePhysicalProperties:
    """
        The class holds the properties of a vehicle.
    """
    
    def __init__(self,width=45,height=25):
        self._width = width
        self._height = height


'''
if __name__ == "__main__": 
    my_prop = VehiclePhysicalProperties(4) 
    my_prop2 = VehiclePhysicalProperties() 
    print(my_prop2._width)
'''