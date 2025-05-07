"""
This program can be launched directly.
To move the drone, you have to click on the map, then use the arrows on the keyboard
"""

import os
import sys
import math
import random
from typing import List, Type
from copy import deepcopy
import arcade

import arcade.key
from spg.utils.definitions import CollisionTypes
from spg.playground import Playground
# This line add, to sys.path, the path to parent path of this file
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from maps.maps_vortex.map_test_04 import My4Map
from spg_overlay.entities.drone_abstract import DroneAbstract, drone_collision_drone, drone_collision_wall
from spg_overlay.entities.normal_wall import NormalWall,SrColorWall
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.reporting.evaluation import ZonesConfig
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle
from spg_overlay.utils.vortex_utils.PID import PID
from spg_overlay.utils.vortex_utils. vortex_state_machines.sensors_analyzer import SensorsAnalyzer
from spg_overlay.utils.vortex_utils.Potential_field import PotentialField

from maps.maps_vortex.generated_code_map04 import add_walls


class MyDroneKeyboard(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.counter = 0
        self.detectionMemorie = []

        self.analyzor = SensorsAnalyzer(signature= "Sensors analyzer",identifier= self.identifier)
        self.analyzor.disable = False


    #COMMUNICATION
    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass
 


    def control(self):
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0,
                   "grasper": 0}


        self.counter += 1
        if self.counter % 50 == 0:
            self.counter = 0
            if self.identifier == 0:
                self.analyzor.analyze(self.lidar_values(), self.lidar_rays_angles(), self.semantic_values())
                print(self.analyzor.analyzed_data["visual connectivity"])
                print(self.analyzor.analyzed_data["positive gap detection memory"])
                #print(self.analyzor.CriticalVisualConnexion(self.lidar_values(), self.analyzor.analyzed_data["drone detection"]))
                

                


        return command
    

def print_keyboard_man():
    print("How to use the keyboard to direct the drone?")
    print("\t- up / down key : forward and backward")
    print("\t- left / right key : turn left / right")
    print("\t- shift + left/right key : left/right lateral movement")
    print("\t- G key : grasp objects")
    print("\t- L key : display (or not) the lidar sensor")
    print("\t- S key : display (or not) the semantic sensor")
    print("\t- P key : draw position from GPS sensor")
    print("\t- C key : draw communication between drones")
    print("\t- M key : print messages between drones")
    print("\t- Q key : exit the program")
    print("\t- R key : reset")
    print("\t- I key : Change controled drone")

def main():
    print_keyboard_man()
    my_map = My4Map()

    playground = my_map.construct_playground(drone_type=MyDroneKeyboard)

    # draw_lidar_rays : enable the visualization of the lidar rays
    # draw_semantic_rays : enable the visualization of the semantic rays
    gui = GuiSR(playground=playground,
                the_map=my_map,
                draw_lidar_rays=True,
                draw_semantic_rays=True,
                use_keyboard=True,
                )
    gui.run()


if __name__ == '__main__':
    main()