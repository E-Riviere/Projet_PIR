import math
import random
from typing import List, Type

from spg.utils.definitions import CollisionTypes
from spg.playground import Playground

from spg_overlay.utils.misc_data import MiscData
from spg_overlay.entities.normal_wall import NormalWall,SrColorWall
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.reporting.evaluation import ZonesConfig
from spg_overlay.entities.drone_abstract import DroneAbstract, drone_collision_drone, drone_collision_wall
from spg_overlay.entities.drone_distance_sensors import DroneLidar

from .generated_code_map07 import add_walls

class My7Map(MapAbstract):

    def __init__(self, zones_config: ZonesConfig = ()):
        super().__init__(zones_config)

        # PARAMETERS MAP
        self._size_area = (1000, 700)

        #POSITIONS OF THE DRONES
        # A fine tuner en fonction du nombre de drone et de la map
        self._number_drones = 2
        # They are positioned in a square whose side size depends on the total number of drones.
        start_pose_drone = (-300, 0.0)
        nb_per_row = math.floor(math.sqrt(float(self._number_drones)))
        nb_per_column = math.ceil(math.sqrt(float(self._number_drones)))
        self.stock_shape = (nb_per_row, nb_per_column)
        dist_inter_drone = 40.0
        # print("nb_per_side", nb_per_side)
        # print("dist_inter_drone", dist_inter_drone)
        # sx = start_area_drones[0] - (nb_per_side - 1) * 0.5 * dist_inter_drone
        # sy = start_area_drones[1] - (nb_per_side - 1) * 0.5 * dist_inter_drone
        # print("sx", sx, "sy", sy)

        self._drones_pos = []
        for i in range(self._number_drones):
            x = start_pose_drone[0] - (float(i) // nb_per_row) * dist_inter_drone
            if i % nb_per_row == 0:
                y = start_pose_drone[1]
            elif i % nb_per_row == 1:
                y = start_pose_drone[1] + 2*dist_inter_drone
            elif i % nb_per_row == 2:
                y = start_pose_drone[1] - 2*dist_inter_drone
            angle = 0.0
            self._drones_pos.append(((x, y), angle))

        self._drones: List[DroneAbstract] = []
        self._number_wounded_persons = 0

    def construct_playground(self, drone_type: Type[DroneAbstract]) -> Playground:
        playground = ClosedPlayground(size=self._size_area)

        add_walls(playground)

        self._explored_map.initialize_walls(playground)

        # POSITIONS OF THE DRONES
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones)
        for i in range(self._number_drones):
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            playground.add(drone, self._drones_pos[i])

        drone_are_invisible_to_lidar = True
        drone_lidar = DroneLidar()
        if drone_are_invisible_to_lidar:
            for agent in playground.agents:
                for sensor in agent.external_sensors:
                    if isinstance(sensor, DroneLidar):
                        for invisible_agent in playground.agents:
                            if agent != invisible_agent:
                                sensor.add_to_temporary_invisible(invisible_agent.base)



        playground.add_interaction(CollisionTypes.PART,
                                   CollisionTypes.ELEMENT,
                                   drone_collision_wall)
        playground.add_interaction(CollisionTypes.PART,
                                   CollisionTypes.PART,
                                   drone_collision_drone)

        return playground
    
    def get_stock_shape(self):
        return (self.stock_shape)
