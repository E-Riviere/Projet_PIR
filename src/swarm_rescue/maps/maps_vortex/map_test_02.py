import math
import random
from typing import List, Type

from spg.playground import Playground
from spg_overlay.entities.normal_wall import NormalWall,SrColorWall
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.reporting.evaluation import ZonesConfig
from spg_overlay.entities.drone_abstract import DroneAbstract

from .generated_code_map02 import add_walls

class MySecondMap(MapAbstract):

    def __init__(self, zones_config: ZonesConfig = ()):
        super().__init__(zones_config)

        # PARAMETERS MAP
        self._size_area = (1000, 700)

    def construct_playground(self, drone_type: Type[DroneAbstract]) -> Playground:
        playground = ClosedPlayground(size=self._size_area)

        add_walls(playground)

        self._explored_map.initialize_walls(playground)

        return playground