import math
import numpy

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule
from spg_overlay.utils.utils import normalize_angle

class SensorsAnalyzer(BrainModule):
    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier
        self.recipient.extend(["situation"])

        self.disable = None
        self.distance_treshold = 70
        self.analyzed_data = {
        "positiv gap number": None,
        "positiv gap index ray": [],
        "positiv gap angle ray": [],
        "positiv gap direction": [],
        "positiv gap detection memory": [],
        "negativ gap number": None,
        "negativ gap index ray": [],
        "negativ gap angle ray": [],
        "negativ gap direction": [],
        "minimum lidar detection": None,
        "maximum lidar detection": None,
        "detection cone": []
        }


    def read_request(self):
        last_request = list(self.recieved_requests.items())[-1]
        if last_request[1] == "sensors analyze":
            self.request(self.signature, "module manager", "Need raw data")
    
    def read_msg(self):
        last_msg = list(self.recieved_msgs.items())[-1]
        if last_msg[1][0] == "sensors raw data":
            self.analyze(last_msg[1], last_msg[2])
            self.send(self.signature, "module manager", ["analyzed data", self.analyzed_data])
    
    def analyze(self, lidar_data, semantic_data):
        self.analyzed_data["positiv gap index ray"] = self.PositiveGapDetector(lidar_data, self.distance_treshold)





    #Calcul raie-angle
    def FromRayToAngle(self, ray, SensorProcess, mirror = False):
        sensor_angle = SensorProcess[1]
        if mirror:
            return(sensor_angle[ray - 1] + 2*math.pi)
        else:
            return(sensor_angle[ray - 1])

    def FromAngleToRay(self, alpha, SensorProcess):
        sensor_angle = SensorProcess[1]
        frame1 = None
        frame2 = None
        for i, angle in enumerate(sensor_angle):
            if alpha == angle:
                return(i + 1)
            if i == len(sensor_angle) - 1:
                return(i + 1)
            elif alpha > angle and alpha < sensor_angle[i+1] :
                frame1 = (i + 1, angle)
                frame2 = (i + 2, sensor_angle[i+1])
                delta1 = abs(alpha - frame1[1])
                delta2 = abs(alpha - frame2[1])
                if delta1 < delta2:
                    return(frame1[0])
                else:
                    return(frame2[0])

    def AngleBetweenRay(self, ray_1, ray_2, LIDARProcess, mirror = False):

        if not mirror:
            angle_ray_1 = self.FromRayToAngle(ray_1, LIDARProcess, False)
            angle_ray_2 = self.FromRayToAngle(ray_2, LIDARProcess, False)
            return abs(normalize_angle(angle_ray_1 - angle_ray_2))
        else:
            angle_ray_1 = self.FromRayToAngle(ray_1, LIDARProcess, False)
            angle_ray_2 = self.FromRayToAngle(ray_2, LIDARProcess, True)
            return abs(angle_ray_1 - angle_ray_2)
        
    def AngleBetweenDir(self, dir_1, dir_2):
        return abs(normalize_angle(dir_1 - dir_2))
    


    # Detection des gap poisitifs
    def PositiveGapDetector(self, LIDARProcess, distance_threshold):

        lidar_data = LIDARProcess[0]
        MaxD = LIDARProcess[2]


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
        return GAP_index_ray
    
    def ComputePositiveGap(self, detection_memorie, LIDARProcess):
        #On calcul l'angle des raies dans le repère du drone
        GAP_size = []
        GAP_angle = []
        GAP_direction = []

        for index in detection_memorie:

            if index[1] > index[2]:
                start_angle = self.FromRayToAngle(index[1], LIDARProcess, False)
                end_angle = self.FromRayToAngle(index[2], LIDARProcess, True)

                GAP_angle.append([start_angle, end_angle])
                GAP_size.append(abs(start_angle - end_angle))
                GAP_direction.append(normalize_angle((start_angle + end_angle)/2))

            else:
                start_angle = self.FromRayToAngle(index[1], LIDARProcess, False)
                end_angle = self.FromRayToAngle(index[2], LIDARProcess, False)

                GAP_angle.append([start_angle, end_angle])
                GAP_size.append(abs(start_angle - end_angle))
                GAP_direction.append(normalize_angle((start_angle + end_angle)/2))

        Inter_pos_size = []

        for i, angle in enumerate(GAP_direction):
            if i < len(GAP_direction) - 1:
                diff_angle = abs(normalize_angle(GAP_direction[i] - GAP_direction[i+1]))
            else:
                diff_angle = abs(normalize_angle(GAP_direction[i] - GAP_direction[0]))
            Inter_pos_size.append(diff_angle)

        return (GAP_angle, GAP_direction, GAP_size, Inter_pos_size)

    #Modification de liste d'etat self.detectionMemorie en fonction des nouveaux gaps detectes 
    def UpdateGapDetection(self, LIDARProcess):
        GAP_index_ray = self.PositiveGapDetector(LIDARProcess)
        counter_match_memorie_gap = 0
        unmatch_memorie_gap = [gap[0] for gap in self.pos_gap_detection_memory]

        for i, indexs in enumerate(GAP_index_ray):

            counter_match_actual_gap = 0
            unmatch_actual_gap = None

            for k, gap in enumerate(self.pos_gap_detection_memory):
                if len(self.pos_gap_detection_memory) != 0:
                    if gap[1] > gap[2]:
                        #print("a")
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
                        #print("a")
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
            #On ajoute les nouveau gap qui n'ont pas match
            if counter_match_actual_gap == 0:
                unmatch_actual_gap = indexs
                num = i
                unmatch_actual_gap.insert(0,i)
                self.pos_gap_detection_memory.insert(num, unmatch_actual_gap)
                self.namearrangement(self.pos_gap_detection_memory, num)
                unmatch_memorie_gap = [m + 1 if i <= m else m for m in unmatch_memorie_gap]
        #On supprime les anciens gap qui n'ont pas match 
        if counter_match_memorie_gap != len(self.pos_gap_detection_memory):
            for m in reversed(unmatch_memorie_gap):
                del (self.pos_gap_detection_memory[m])
                self.namearrangement(self.pos_gap_detection_memory, m - 1)

    #Rearrangement du nommage des gaps en cas de disparition ou d'ajout de gap à liste d'etat self.detectionMemorie
    def namearrangement(self, detection_state, name):
        if len(detection_state) == 1:
            detection_state[name][0] = 0
            return(detection_state)
        if name < len(detection_state) - 1:
            if name == -1:
                name = 0
                detection_state[name][0] = 0
                return(self.namearrangement(detection_state,name))
            if detection_state[name+1][0] != detection_state[name][0] + 1:
                detection_state[name+1][0] += -(detection_state[name+1][0] - (detection_state[name][0] + 1))
                return(self._namearrangement(detection_state, name + 1))
            else:
                return(self.namearrangement(detection_state, name + 1))



    #detection des gap negatifs
    def NegativeGapDetector(self, LIDARProcess):

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
                        Obst_dist_ray.append([lidar_data[start_ray - 1], min_neg_gap, lidar_data[end_ray - 1]])
                        start_ray = None
                        min_ray = None
                if i == len(lidar_data) - 1 and lidar_data[-1] < distance_threshold :
                    end_ray = len(lidar_data)
                    Obst_index_ray.append([start_ray,min_ray, end_ray])
                    Obst_dist_ray.append([lidar_data[start_ray - 1], min_neg_gap, lidar_data[end_ray - 1]])

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
                
                start_angle = self.FromRayToAngle(index[0], LIDARProcess, False)
                min_angle = self.FromRayToAngle(index[1], LIDARProcess, False)
                end_angle = self.FromRayToAngle(index[2], LIDARProcess, False)

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
    

        #drone detection
    def DroneSemanticDetection(self, SEMANTICProcess, LIDARProcess):

        resolution = abs(SEMANTICProcess[1][0] - SEMANTICProcess[1][1])
        drone_angle_ray = SEMANTICProcess[2]

        drone_index_ray_on_lidar = []
        start_ray = None
        end_ray = None

        for i, angle in enumerate(drone_angle_ray):
            if i == len(drone_angle_ray) - 1:
                end_ray = self.FromAngleToRay(angle, LIDARProcess)
                if start_ray != None:
                    drone_index_ray_on_lidar.append([start_ray, end_ray])
                else:
                    drone_index_ray_on_lidar.append([end_ray, end_ray])
            elif abs(drone_angle_ray[i+1] - drone_angle_ray[i]) < 0.19: #resolution semantic sensor
                if start_ray == None:
                    start_ray = self.FromAngleToRay(angle,LIDARProcess)
            else:
                if start_ray != None:
                    end_ray = self.FromAngleToRay(angle,LIDARProcess)
                    drone_index_ray_on_lidar.append([start_ray, end_ray])
                    start_ray = None
                else:
                    start_ray = self.FromAngleToRay(angle,LIDARProcess)
                    drone_index_ray_on_lidar.append([start_ray, start_ray])
                    start_ray = None

        detection_cone = []
        for i,frame in enumerate(drone_index_ray_on_lidar):
            extended_frame0 = frame[0]
            if frame[0] > 1:
                angle = self.FromRayToAngle(frame[0], LIDARProcess) - 2 * resolution
                if angle < -math.pi * 2:
                    angle = -math.pi * 2
                frame[0] = self.FromAngleToRay(angle, LIDARProcess)
                extended_frame0 = frame[0]
            extended_frame1 = frame[1]
            if frame[1] < 181:
                angle = self.FromRayToAngle(frame[1], LIDARProcess) + 2 * resolution
                if angle > math.pi * 2:
                    angle = math.pi * 2
                frame[1] = self.FromAngleToRay(angle, LIDARProcess)
                extended_frame1 = frame[1]
            detection_cone.append([extended_frame0, extended_frame1])
        #print(detection_cone)
        return detection_cone


    def VisualConnexion(self, LIDARProcess, SEMANTICProcess, detection_cone):
        lidar_data = LIDARProcess[0]
        drone_dist = SEMANTICProcess[3]
        if len(drone_dist) > 0:
            obst_detection = False
            mean_dist = numpy.mean(drone_dist)
            for frame in detection_cone:
                for i in range(frame[0], frame[1] + 1):
                    if lidar_data[i - 1] < mean_dist:
                        #print("obst", self.identifier)
                        self.critical_visual_connectivity = True
                        obst_detection = True

            if not obst_detection:
                self.critical_visual_connectivity = False
            if mean_dist > 150:
                self.critical_visual_connectivity = True
        else:
            self.visual_connectivity = False
            self.critical_visual_connectivity = False

