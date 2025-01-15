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

from maps.maps_vortex.generated_code_map04 import add_walls


class MyDroneKeyboard(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.counter = 0
        self.detectionMemorie = []

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def process_lidar_sensor(self):

            lidar_data = self.lidar_values()
            MaxD = max(lidar_data)

            if lidar_data is None:
                return False
            
            distance_ray_45 = lidar_data[44]
            distance_ray_46 = lidar_data[45]
            distance_ray_135 = lidar_data[134]
            distance_ray_136 = lidar_data[135]

            Placement_Ray = (distance_ray_45, distance_ray_46, distance_ray_135, distance_ray_136)

            distance_ray_130 = lidar_data[129]
            distance_ray_140 = lidar_data[139]
            Orientation_Ray = (distance_ray_130, distance_ray_140)

            return((lidar_data, MaxD, Placement_Ray, Orientation_Ray))


    def process_semantic_sensor(self):

        semantic_data = self.semantic_values()
        return(semantic_data)


    def _FromRayToAngle(self, ray, mirror = False):
        if mirror:
            return ((360/181)*(2*math.pi/360)*ray + math.pi)
        if ray == 181:
            return ((360/181)*(2*math.pi/360)*ray - math.pi)
        else:
            return normalize_angle(((360/181)*(2*math.pi/360)*ray - math.pi))


    def _FromAngleToRay(self, alpha):
        ray = (181/(2 * math.pi)) * (alpha + math.pi)
        if ray % 1 < 0.5:
            ray = math.floor(ray)
        else:
            ray = math.ceil(ray)
        return (ray)
    

    def _AngleBetweenRay(self, ray_1, ray_2, mirror = False):
        if not mirror:
            angle_ray_1 = self._FromRayToAngle(ray_1, False)
            angle_ray_2 = self._FromRayToAngle(ray_2, False)
            return abs(normalize_angle(angle_ray_1 - angle_ray_2))
        else:
            angle_ray_1 = self._FromAngleToRay(ray_1, False)
            angle_ray_2 = self._FromAngleToRay(ray_2, True)
            return abs(angle_ray_1 - angle_ray_2)
        

    def _AngleBetweenDir(self, dir_1, dir_2):
        return abs(normalize_angle(dir_1 - dir_2))
    

    def _OrientationToWall(self,ray_1, ray_2, dist_ray_1, dist_ray_2):

        diff_angle = self._AngleBetweenRay(ray_1, ray_2, False)

        wall_angle = math.atan((dist_ray_1 * math.cos(diff_angle) - dist_ray_2)/(dist_ray_1 * math.sin(diff_angle)))

        return wall_angle


    def _PositiveGapDetector(self,LIDARProcess):
    
        lidar_data = LIDARProcess[0]
        MaxD = LIDARProcess[1]

        #distance_threshold = 0.5 * MaxD
        distance_threshold = 80

        GAP_index_ray = []
        start_ray = None
        end_ray = None

        #On récupère l'indice de la raie qui débute le gap et l'indice de la raie qui termine le gap
        for i, dist in enumerate(lidar_data):
            if dist > distance_threshold:
                if start_ray == None:
                    start_ray = i + 1
                
            if dist < distance_threshold:
                if start_ray != None: 
                    end_ray = i
                    GAP_index_ray.append([start_ray, end_ray])
                    start_ray = None

            if i == len(lidar_data) - 1 and lidar_data[-1] > distance_threshold:
                end_ray = len(lidar_data)
                GAP_index_ray.append([start_ray, end_ray])

        #On retire les gaps trop petit
        for i in range(len(GAP_index_ray) - 1, -1, -1):
            if i == 0 or i == len(GAP_index_ray) - 1: 
                if GAP_index_ray[0][0] == 1 and GAP_index_ray[-1][1] == 181:
                    if abs(GAP_index_ray[0][0] - GAP_index_ray[0][1]) + abs(GAP_index_ray[-1][0] - GAP_index_ray[-1][1]) < 17:
                        del GAP_index_ray[i]
                elif abs(GAP_index_ray[i][0] - GAP_index_ray[i][1]) < 17:
                    del (GAP_index_ray[i])

            elif abs(GAP_index_ray[i][0] - GAP_index_ray[i][1]) < 17:
                del (GAP_index_ray[i])

        #On concatène les gaps qui termine et commence un tour complet
        if len(GAP_index_ray) > 1:
            if GAP_index_ray[-1][-1] == 181 and GAP_index_ray[0][0] == 1:
                GAP_index_ray[0][0] = GAP_index_ray[-1][0]
                del(GAP_index_ray[-1])
        
        #Modification de liste d'etat self.detectionMemorie en fonction des nouveaux gaps detectes 
                
        counter_match_memorie_gap = 0
        unmatch_memorie_gap = [gap[0] for gap in self.detectionMemorie]
        # print("e", self.detectionMemorie)
        # print(GAP_index_ray)
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
                        #print("b")
                        if indexs[0] <= gap[2] and indexs[1] >= gap[1]:
                            if counter_match_actual_gap == 0:
                                gap[1:3] = indexs
                                counter_match_actual_gap += 1 
                                counter_match_memorie_gap += 1
                                unmatch_memorie_gap.remove(k)
            #print("e",unmatch_memorie_gap)
            if counter_match_actual_gap == 0:
                unmatch_actual_gap = indexs
                num = i
                unmatch_actual_gap.insert(0,i)
                self.detectionMemorie.insert(num, unmatch_actual_gap)
                self._namearrangement(self.detectionMemorie, num)
                unmatch_memorie_gap = [m + 1 if i <= m else m for m in unmatch_memorie_gap]
            #print("s", unmatch_memorie_gap)
        if counter_match_memorie_gap != len(self.detectionMemorie):
            for m in reversed(unmatch_memorie_gap):
                del (self.detectionMemorie[m])
                self._namearrangement(self.detectionMemorie, m - 1)
        #print("s", self.detectionMemorie)

    
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

    def _ComputePositiveGap(self, detection_memorie):
        #On calcul l'angle des raies dans le repère du drone
        GAP_size = []
        GAP_angle = []
        GAP_direction = []

        for index in detection_memorie:

            if index[1] > index[2]:
                start_angle = (360/181)*(2*math.pi/360)*index[1] - math.pi
                end_angle = (360/181)*(2*math.pi/360)*index[2] + math.pi

                GAP_angle.append([start_angle, normalize_angle(end_angle)])
                GAP_size.append(abs(start_angle - end_angle))
                GAP_direction.append(normalize_angle((start_angle + end_angle)/2))

            else:
                if index[2] == 181:
                    end_angle = ((360/181)*(2*math.pi/360)*index[2] - math.pi)
                else:
                    end_angle = normalize_angle(((360/181)*(2*math.pi/360)*index[2] - math.pi))
                start_angle = normalize_angle(((360/181)*(2*math.pi/360)*index[1] - math.pi))

                GAP_angle.append([start_angle, end_angle])
                GAP_size.append(abs(start_angle - end_angle))
                GAP_direction.append(normalize_angle((start_angle + end_angle)/2))
        #On calcul l'angle entre les raies de distance minimum de deux négatif
        Inter_pos_size = []

        for i, angle in enumerate(GAP_direction):
            if i < len(GAP_direction) - 1:
                diff_angle = abs(normalize_angle(GAP_direction[i] - GAP_direction[i+1]))
            else:
                diff_angle = abs(normalize_angle(GAP_direction[i] - GAP_direction[0]))
            Inter_pos_size.append(diff_angle)

        return (GAP_angle, GAP_direction, GAP_size, Inter_pos_size)


    def _NegativeGapDetector(self,LIDARProcess):

        lidar_data = LIDARProcess[0]

        distance_threshold = 80
        Obst_index_ray = []
        Obst_dist_ray = []

        start_ray = None
        end_ray = None
        min_ray = None
        #On récupère l'indice de la raie qui débute le gap et l'indice de la raie qui termine le gap

        for i, dist in enumerate(lidar_data):
            if dist < distance_threshold:
                if start_ray == None:
                    start_ray = i + 1
                    min_neg_gap = dist
            if start_ray != None:
                if lidar_data[i] <= min_neg_gap:
                    min_neg_gap = lidar_data[i]
                    min_ray = i + 1
            if dist > distance_threshold:
                if start_ray != None:
                    end_ray = i
                    Obst_index_ray.append([start_ray,min_ray, end_ray])
                    Obst_dist_ray.append([lidar_data[start_ray-1], min_neg_gap, lidar_data[end_ray -1]])
                    start_ray = None
                    min_ray = None
            if i == len(lidar_data) - 1 and lidar_data[-1] < distance_threshold :
                end_ray = len(lidar_data)
                Obst_index_ray.append([start_ray,min_ray, end_ray])
                Obst_dist_ray.append([lidar_data[start_ray-1], min_neg_gap, lidar_data[end_ray -1]])

        #On retire les gaps trop petit
        for i in range(len(Obst_index_ray) - 1, -1, -1):
            if i == 0 or i == len(Obst_index_ray) - 1: 
                if Obst_index_ray[0][0] == 1 and Obst_index_ray[-1][1] == 181:
                    if abs(Obst_index_ray[0][0] - Obst_index_ray[0][2]) + abs(Obst_index_ray[-1][0] - Obst_index_ray[-1][2]) < 5:
                        del Obst_index_ray[i]
                        del Obst_dist_ray[i]
                elif abs(Obst_index_ray[i][0] - Obst_index_ray[i][2]) < 5:
                    del (Obst_index_ray[i])
                    del Obst_dist_ray[i]

            elif abs(Obst_index_ray[i][0] - Obst_index_ray[i][2]) < 5:
                del (Obst_index_ray[i])
                del Obst_dist_ray[i]



        #On concatène les gaps qui termine et commence un tour complet
        if len(Obst_index_ray) > 1:
            if Obst_index_ray[-1][-1] == 181 and Obst_index_ray[0][0] == 1:
                Obst_index_ray[0][0] = Obst_index_ray[-1][0]
                Obst_dist_ray[0][0] = Obst_dist_ray[-1][0]
                Obst_dist_ray[0][1] = Obst_dist_ray[-1][1]
                del(Obst_index_ray[-1])
                del(Obst_dist_ray[-1])

        #On récupère les angles des raies
        Obst_angle_ray =[]

        for index in Obst_index_ray:
            
            start_angle = self._FromRayToAngle(index[0], False)
            min_angle = self._FromRayToAngle(index[1], False)
            end_angle = self._FromRayToAngle(index[2], False)

            Obst_angle_ray.append([start_angle, min_angle, end_angle])

        #On calcul l'angle entre les raies de distance minimum de deux négatif
        Inter_neg_size = []

        for i, angle in enumerate(Obst_angle_ray):
            if i < len(Obst_angle_ray) - 1:
                diff_angle = abs(normalize_angle(Obst_angle_ray[i][1] - Obst_angle_ray[i+1][1]))
            else:
                diff_angle = abs(normalize_angle(Obst_angle_ray[i][1] - Obst_angle_ray[0][1]))
            Inter_neg_size.append(diff_angle)


        return ((Obst_index_ray, Obst_dist_ray, Obst_angle_ray, Inter_neg_size))


    def control(self):
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0,
                   "grasper": 0}
        self.counter += 1

        LIDARProcess = self.process_lidar_sensor()
        PositiveDetection = self._PositiveGapDetector(LIDARProcess)
        NegativeDetection = self._NegativeGapDetector(LIDARProcess)

        GAP_analysis = self._ComputePositiveGap(self.detectionMemorie)
        #print(Classifier)
        if self.counter % 50 == 0:
            self.counter = 0
            #print(LIDARProcess[3])
            # print(Agent_angle)
            #print(NegativeDetection[0], NegativeDetection[1])
            #print(PositiveDetection[0])
            #print(Classifier)
            #print(self.detectionMemorie)
            #print(GAP_analysis[1], GAP_analysis[3])
            #print(Wall_Orientation)
            print(self.process_semantic_sensor())
            #print(self.process_lidar_sensor())


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