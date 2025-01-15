import math
import random
import numpy
from typing import Optional
import time

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle
from spg_overlay.utils.vortex_utils.PID import PID
from spg_overlay.utils.vortex_utils.ray_analyse import Analyzer
from spg_overlay.utils.vortex_utils.Potential_field import PotentialField
from spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor
from spg_overlay.entities.drone_distance_sensors import compute_ray_angles


class MyDroneVortex(DroneAbstract):
    def __init__(self,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        super().__init__(identifier=identifier,
                         misc_data=misc_data,
                         display_lidar_graph=False,
                         **kwargs)
        
        # if identifier is None:
        #     identifier = id(self)
        self.identifier = identifier

    def define_message_for_all(self):
        pass


    def control(self):

        forward = 0.0
        lateral = 0.0
        rotation = 0.0

        if self.identifier == 7:
            forward = 1.0

        command ={"forward" : forward,
                  "lateral" : lateral,
                  "rotation" : rotation
                  }
        
        return command
    

