import math
import random
from typing import List, Type

from spg.playground import Playground
from spg_overlay.entities.normal_wall import NormalWall,SrColorWall
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.reporting.evaluation import ZonesConfig
from spg_overlay.entities.drone_abstract import DroneAbstract


class MyFirstMap(MapAbstract):

    def __init__(self, zones_config: ZonesConfig = ()):
        super().__init__(zones_config)

        # PARAMETERS MAP
        self._size_area = (1000, 700)


    def construct_playground(self, drone_type: Type[DroneAbstract]) -> Playground:
        playground = ClosedPlayground(size=self._size_area)

        wall1 = SrColorWall (pos_start=(-400,50),pos_end=(-100,50),width = 6, color = (0,0,0))
        playground.add(wall1, wall1.wall_coordinates)

        wall2 = SrColorWall (pos_start=(-400,-50),pos_end=(300,-50),width = 6, color = (0,0,0))
        playground.add(wall2, wall2.wall_coordinates)

        wall3 = SrColorWall (pos_start=(-100,50),pos_end=(-100,200),width = 6, color = (0,0,0))
        playground.add(wall3, wall3.wall_coordinates)

        wall4 = SrColorWall (pos_start=(0,200),pos_end=(0,50),width = 6, color = (0,0,0))
        playground.add(wall4, wall4.wall_coordinates)

        wall5 = SrColorWall (pos_start=(-100,200),pos_end=(0,200),width = 6, color = (0,0,0))
        playground.add(wall5, wall5.wall_coordinates)

        wall6 = SrColorWall (pos_start=(0,50),pos_end=(300,50),width = 6, color = (0,0,0))
        playground.add(wall6, wall6.wall_coordinates)

        wall7 = SrColorWall (pos_start=(300,50),pos_end=(300,-50),width = 6, color = (0,0,0))
        playground.add(wall7, wall7.wall_coordinates)

        self._explored_map.initialize_walls(playground)

        return playground
