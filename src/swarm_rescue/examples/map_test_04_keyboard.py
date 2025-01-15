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
from spg_overlay.utils.vortex_utils.ray_analyse import Analyzer
from spg_overlay.utils.vortex_utils.Potential_field import PotentialField

from maps.maps_vortex.generated_code_map04 import add_walls


class MyDroneKeyboard(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.counter = 0
        self.detectionMemorie = []
        self.analyzer = Analyzer()


    #COMMUNICATION
    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass



    #SEMANTIC SNESOR PROCESS
    def process_semantic_sensor(self):

        semantic_data = self.semantic_values()
        return(semantic_data)


    def _ComportementSelector(self, semantic_data):
        if len(semantic_data) == 0:
            self.isFollower = False
            self.isExplorator = True
        else:
            for data in semantic_data:
                if data.angle > -math.pi/2 and data.angle < math.pi/2:
                    self.isFollower = True
                    self.isExplorator = False
            if not self.isFollower:
                self.isFollower = False
                self.isExplorator = True     

    # def _UpdateDroneLidarDetection(self, semantic_data):
    #     drone_index_ray = []
    #     for data, i in enumerate(semantic_data):
    #         angle = 




    #LIDAR SENSOR PROCESS
    def process_lidar_sensor(self):

        lidar_data = self.lidar_values()
        lidar_angle = self.lidar_rays_angles()
        MaxD = max(lidar_data)
        MinD = min(lidar_data)

        if lidar_data is None:
            return False

        return((lidar_data, lidar_angle, MaxD, MinD))
    

    # def _OrientationToWall(self,ray_1, ray_2, dist_ray_1, dist_ray_2):

    #     diff_angle = self.analyze.AngleBetweenRay(ray_1, ray_2, False)

    #     wall_angle = math.atan((dist_ray_1 * math.cos(diff_angle) - dist_ray_2)/(dist_ray_1 * math.sin(diff_angle)))

    #     return wall_angle


    #Collision
    def _CollideDetection(self, LIDARProcess):
        Obst = []
        if LIDARProcess[3] < 20:
            self.isCollided = True
        else:
            self.isCollided = False
        for i, dist in enumerate(LIDARProcess[0]):
            if dist < 20:
                Obst.append([i, dist])
        return Obst
    

    #Modification de liste d'etat self.detectionMemorie en fonction des nouveaux gaps detectes 
    def _UpdateGapDetection(self, LIDARProcess):
        GAP_index_ray = self.analyzer.PositiveGapDetector(LIDARProcess)
        counter_match_memorie_gap = 0
        unmatch_memorie_gap = [gap[0] for gap in self.detectionMemorie]

        for i, indexs in enumerate(GAP_index_ray):

            counter_match_actual_gap = 0
            unmatch_actual_gap = None

            for k, gap in enumerate(self.detectionMemorie):
                if len(self.detectionMemorie) != 0:
                    if gap[1] > gap[2]:
                        if indexs[0] > indexs[1]:
                            if indexs[0] <= gap[2] + 181 and indexs[1] + 181 >= gap[1]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)
                        if indexs[1] < 90:
                            if indexs[0] + 181 <= gap[2] + 181 and indexs[1] + 181 >= gap[1]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)
                        if indexs[0] > 90:
                            if indexs[0] <= gap[2] + 181 and indexs[1] >= gap[1]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)

                    if indexs[0] > indexs[1]:
                        if gap[1]> gap[2]:
                            if gap[1] <= indexs[1] + 181 and gap[2] + 181 >= indexs[0]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)
                        if gap[2] < 90:
                            if gap[1] + 181 <= indexs[1] + 181 and gap[2] + 181 >= indexs[0]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)
                        if gap[1] > 90:
                            if gap[1] <= indexs[1] + 181 and gap[2] >= indexs[0]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)

                    else:
                        if indexs[0] <= gap[2] and indexs[1] >= gap[1]:
                            if counter_match_actual_gap == 0:
                                gap[1:3] = indexs
                                counter_match_actual_gap += 1 
                                counter_match_memorie_gap += 1
                                unmatch_memorie_gap.remove(k)
            #On ajoute les nouveau gap qui n'ont pas match
            if counter_match_actual_gap == 0:
                unmatch_actual_gap = indexs
                num = i
                unmatch_actual_gap.insert(0,i)
                self.detectionMemorie.insert(num, unmatch_actual_gap)
                self._namearrangement(self.detectionMemorie, num)
                unmatch_memorie_gap = [m + 1 if i <= m else m for m in unmatch_memorie_gap]
        #On supprime les anciens gap qui n'ont pas match 
        if counter_match_memorie_gap != len(self.detectionMemorie):
            for m in reversed(unmatch_memorie_gap):
                del (self.detectionMemorie[m])
                self._namearrangement(self.detectionMemorie, m - 1)

    #Rearrangement du nommage des gaps en cas de disparition ou d'ajout de gap à liste d'etat self.detectionMemorie
    def _namearrangement(self, detection_state, name):
        if len(detection_state) == 1:
            detection_state[name][0] = 0
            return(detection_state)
        if name < len(detection_state) - 1:
            if name == -1:
                name = 0
                detection_state[name][0] = 0
                return(self._namearrangement(detection_state,name))
            if detection_state[name+1][0] != detection_state[name][0] + 1:
                detection_state[name+1][0] += -(detection_state[name+1][0] - (detection_state[name][0] + 1))
                return(self._namearrangement(detection_state, name + 1))
            else:
                return(self._namearrangement(detection_state, name + 1))


    def _GapCounter(self):
        return(len(self.detectionMemorie))


    def _AgentSituation(self, Detection, gap_number):

        GAP_size = Detection[2]
        Inter_gap = Detection[3]

        if gap_number == 1:

            if GAP_size[0] > math.pi * 2/3:
                #print("O")
                self.isInOpenSpace = True
                self.isInIntersection = False
                self.isInCorridor = False
                self.isInDeadEnd = False
                self.isInCurve =False
            
            else:
                #print("D")
                self.isInOpenSpace = False
                self.isInIntersection = False
                self.isInCorridor = False
                self.isInDeadEnd = True
                self.isInCurve = False

        if gap_number == 2:
            
            if GAP_size[0] > math.pi * 2/3 or GAP_size[1] > math.pi * 2/3:
                #print("O")
                self.isInOpenSpace = True
                self.isInIntersection = False
                self.isInCorridor = False
                self.isInDeadEnd = False
                self.isInCurve = False

            elif Inter_gap[0] < math.pi * 3/4:
                #print("V")
                self.isInOpenSpace = False
                self.isInIntersection = False
                self.isInCorridor = False
                self.isInDeadEnd = False
                self.isInCurve = True
    
            else:
                #print("C")
                self.isInOpenSpace = False
                self.isInIntersection = False
                self.isInCorridor = True
                self.isInDeadEnd = False
                self.isInCurve = False

        if gap_number > 2:
            print("I")
            self.isInOpenSpace = False
            self.isInIntersection = True
            self.isInCorridor = False
            self.isInDeadEnd = False
            self.isInCurve = False
    

    def _GapSelector(self, detection_memorie) :
        
        if self.isInOpenSpace:
            self.gapSelected = detection_memorie[-1][0]

        elif self.isInCorridor:
            self.gapSelected = detection_memorie[-1][0]

        elif self.isInIntersection:
            self.gapSelected = detection_memorie[-1][0]

        elif self.isInCurve:
            self.gapSelected = detection_memorie[1][0]

        elif self.isInDeadEnd:
            self.gapSelected = detection_memorie[0][0]
        
        else:
            return 0


    def control(self):
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0,
                   "grasper": 0}
        self.counter += 1

        # LIDARProcess = self.process_lidar_sensor()
        # PositiveDetection = self.analyzer.PositiveGapDetector(LIDARProcess)
        # NegativeDetection = self.analyzer.NegativeGapDetector(LIDARProcess)
        # self._UpdateGapDetection(LIDARProcess)
        # GAP_analysis = self.analyzer.ComputePositiveGap(self.detectionMemorie, LIDARProcess)
        #Wall_Orientation = self._OrientationToWall(NegativeDetection[0][0][0], NegativeDetection[0][0][1], NegativeDetection[1][0][0], NegativeDetection[1][0][1])
        #print(Classifier)
        if self.counter % 50 == 0:
            self.counter = 0
            #print(LIDARProcess[3])
            # print(Agent_angle)
            # print("a", NegativeDetection[0], NegativeDetection[1])
            #print(PositiveDetection[0])
            #print(Classifier)
            # print(self.detectionMemorie)
            #print(GAP_analysis[1], GAP_analysis[3])
            #print(Wall_Orientation)

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