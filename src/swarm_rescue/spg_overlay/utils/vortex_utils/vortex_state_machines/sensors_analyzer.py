import math
import numpy

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule
from spg_overlay.utils.utils import normalize_angle

from spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor
from spg_overlay.entities.drone_distance_sensors import compute_ray_angles


#Calcul raie-angle
def FromRayToAngle(ray, sensor_angle, mirror = False):
    if mirror:
        return(sensor_angle[ray - 1] + 2*math.pi)
    else:
        return(sensor_angle[ray - 1])

def FromAngleToRay(alpha, sensor_angle):
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

def AngleBetweenRay(ray_1, ray_2, ray_angles, mirror = False):

    if not mirror:
        angle_ray_1 = FromRayToAngle(ray_1, ray_angles, False)
        angle_ray_2 = FromRayToAngle(ray_2, ray_angles, False)
        return abs(normalize_angle(angle_ray_1 - angle_ray_2))
    else:
        angle_ray_1 = FromRayToAngle(ray_1, ray_angles, False)
        angle_ray_2 = FromRayToAngle(ray_2, ray_angles, True)
        return abs(angle_ray_1 - angle_ray_2)
    
def AngleBetweenDir(dir_1, dir_2):
    return abs(normalize_angle(dir_1 - dir_2))

class SensorsAnalyzer(BrainModule):
    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier
        self.recipient.extend(["situation"])

        self.disable = True
        self.distance_treshold = 70
        self.analyzed_data = {
        "positive gap number": None,
        "positive gap index ray": [],
        "positive gap angle ray": [],
        "positive gap direction": [],
        "positive gap detection memory": [],
        "negative gap number": None,
        "negative gap index ray": [],
        "negative gap dist ray" : [],
        "negative gap angle ray": [],
        "minimum lidar detection": None,
        "maximum lidar detection": None,
        "drone detection" : None,
        "visual connectivity" : [],
        "collision" : []
        }

        self.recieved_requests = {
        "Need sensors analyze" : None
        }

        self.recieved_msgs = {
        "sensors raw data" : None
        }


    def read_request(self, request):
        if request == "Need sensors analyze":
            self.disable = False
            self.request(self.signature, "Module manager", "Need raw data")
    
    def read_msg(self, title):
        dico = self.recieved_msgs[title][1]
        if title == "sensors raw data":
            self.analyze(dico["lidar data"], dico["lidar ray angles"], dico["semantic data"])
            self.send(self.signature, "Module manager", "analyzed data", self.analyzed_data)

    def analyze(self, lidar_data, lidar_ray_angles, semantic_data):
        if self.disable == False:
            self.analyzed_data["positive gap index ray"] = self.PositiveGapDetector(lidar_data, self.distance_treshold)
            self.UpdateGapDetection(self.analyzed_data["positive gap index ray"])
            self.analyzed_data["positive gap number"] = len(self.analyzed_data["positive gap detection memory"])
            self.analyzed_data["positive gap angle ray"], self.analyzed_data["positive gap direction"] = self.ComputePositiveGap(self.analyzed_data["positive gap detection memory"],lidar_ray_angles)[:2]
            self.analyzed_data["negative gap index ray"], self.analyzed_data["negative gap dist ray"], self.analyzed_data["negative gap angle ray"] = self.NegativeGapDetector(lidar_data, lidar_ray_angles)[:3]
            self.analyzed_data["negative gap number"] = len(self.analyzed_data["negative gap index ray"])
            self.analyzed_data["minimum lidar detection"] = min(lidar_data)
            self.analyzed_data["maximum lidar detection"] = max(lidar_data)
            self.analyzed_data["drone detection"] = self.DroneSemanticDetection(semantic_data)
            self.analyzed_data["visual connectivity"] = self.visual_connectvity_list(self.analyzed_data["positive gap detection memory"],
                                                                                     self.analyzed_data["drone detection"],
                                                                                     self.CriticalVisualConnexion(lidar_data, self.analyzed_data["drone detection"]))
            self.analyzed_data["collision"] = self.CollideDetection(lidar_data)

    #Collision
    def CollideDetection(self, lidar_data):
        Obst = []
        if min(lidar_data) < 20:
            for i, dist in enumerate(lidar_data):
                if dist < 20:
                    Obst.append([i, dist])
            return Obst

        else:
            pass



    # Detection des gap poisitifs
    def PositiveGapDetector(self, lidar_data, distance_threshold):

        # lidar_data = LIDARProcess#[0] 
        # MaxD = LIDARProcess[2]


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
    
    def ComputePositiveGap(self, detection_memorie, ray_angles):
        #On calcul l'angle des raies dans le repère du drone
        GAP_size = []
        GAP_angle = []
        GAP_direction = []

        for index in detection_memorie:

            if index[1] > index[2]:
                start_angle = FromRayToAngle(index[1], ray_angles, False)
                end_angle = FromRayToAngle(index[2], ray_angles, True)

                GAP_angle.append([start_angle, end_angle])
                GAP_size.append(abs(start_angle - end_angle))
                GAP_direction.append(normalize_angle((start_angle + end_angle)/2))

            else:
                start_angle = FromRayToAngle(index[1], ray_angles, False)
                end_angle = FromRayToAngle(index[2], ray_angles, False)

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
    def UpdateGapDetection(self, GAP_index_ray):
        counter_match_memorie_gap = 0
        unmatch_memorie_gap = [gap[0] for gap in self.analyzed_data["positive gap detection memory"]]

        for i, indexs in enumerate(GAP_index_ray):

            counter_match_actual_gap = 0
            unmatch_actual_gap = None

            for k, gap in enumerate(self.analyzed_data["positive gap detection memory"]):
                if len(self.analyzed_data["positive gap detection memory"]) != 0:
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
                self.analyzed_data["positive gap detection memory"].insert(num, unmatch_actual_gap)
                self.namearrangement(self.analyzed_data["positive gap detection memory"], num)
                unmatch_memorie_gap = [m + 1 if i <= m else m for m in unmatch_memorie_gap]
        #On supprime les anciens gap qui n'ont pas match 
        if counter_match_memorie_gap != len(self.analyzed_data["positive gap detection memory"]):
            for m in reversed(unmatch_memorie_gap):
                del (self.analyzed_data["positive gap detection memory"][m])
                self.namearrangement(self.analyzed_data["positive gap detection memory"], m - 1)

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
                return(self.namearrangement(detection_state, name + 1))
            else:
                return(self.namearrangement(detection_state, name + 1))



    #detection des gap negatifs
    def NegativeGapDetector(self, lidar_data, ray_angles):

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
                
                start_angle = FromRayToAngle(index[0], ray_angles, False)
                min_angle = FromRayToAngle(index[1], ray_angles, False)
                end_angle = FromRayToAngle(index[2], ray_angles, False)

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
    def DroneSemanticDetection(self, semantic_data):
        
        semantic_angles = compute_ray_angles(2*math.pi, 181)
 
        drone_angle_ray = [] 
        drone_dist_ray = []
        drone_id_ray = []         
        drone_index_ray = []

        for i, data in enumerate(semantic_data):
            if data.entity_type == DroneSemanticSensor.TypeEntity.DRONE:
                drone_angle_ray.append(data.angle)
                drone_dist_ray.append(data.distance)
                drone_id_ray.append(data.identifier)
                drone_index_ray.append(FromAngleToRay(data.angle, semantic_angles))

        drone_detection = {str(id):[[],[],[],[],[],[],[]] for id in drone_id_ray}
        for i, id in enumerate(drone_id_ray):
            drone_detection[str(id)][0].append(int(drone_index_ray[i]))
            drone_detection[str(id)][1].append(float(drone_angle_ray[i]))
            drone_detection[str(id)][2].append(float(drone_dist_ray[i]))

        for id in reversed(list(drone_detection.keys())):
            if len(drone_detection[id][0]) < 3:
                del(drone_detection[id])

        for id in drone_detection.keys():
                drone_detection[id][3] = float(numpy.mean(drone_detection[id][2]))
                if drone_detection[id][0][0] <= 2 and drone_detection[id][0][-1] >= 180:
                    drone_detection[id][5] = (int(min(index for index in drone_detection[id][0] if index > 90)),
                                                int(max(index for index in drone_detection[id][0] if index < 90)))
                    start_angle = FromRayToAngle(drone_detection[id][5][0], semantic_angles, False)
                    end_angle = FromRayToAngle(drone_detection[id][5][1], semantic_angles, True)
                    drone_detection[id][4] = normalize_angle((start_angle + end_angle)/2)
                else:
                    drone_detection[id][5] = (drone_detection[id][0][0], drone_detection[id][0][-1])
                    start_angle = FromRayToAngle(drone_detection[id][5][0], semantic_angles, False)
                    end_angle = FromRayToAngle(drone_detection[id][5][1], semantic_angles, False)
                    drone_detection[id][4] = normalize_angle((start_angle + end_angle)/2)
        
        self.DetectionCone(drone_detection)        
            
        return drone_detection

    def DetectionCone(self, drone_detection):
        # Detection cone modifie directement l'argument drone detection stocké dans analyzed data!
        semantic_angles = compute_ray_angles(2*math.pi, 181)
        resolution = abs(semantic_angles[0] - semantic_angles[1])

        for id, detection in drone_detection.items():
            frame = list(detection[5])
            extended_frame0 = frame[0]
            if frame[0] > 1:
                angle = FromRayToAngle(frame[0], semantic_angles) - 2 * resolution
                if angle < -math.pi * 2:
                    angle = -math.pi * 2
                frame[0] = FromAngleToRay(angle, semantic_angles)
                extended_frame0 = frame[0]
            extended_frame1 = frame[1]
            if frame[1] < 181:
                angle = FromRayToAngle(frame[1], semantic_angles) + 2 * resolution
                if angle > math.pi * 2:
                    angle = math.pi * 2
                frame[1] = FromAngleToRay(angle, semantic_angles)
                extended_frame1 = frame[1]

            drone_detection[id][6]=[extended_frame0, extended_frame1]


    def CriticalVisualConnexion(self, lidar_data, drone_detection):
        CVC = {"CVC obst":[], "CVC dist":[]}

        for id, detection in drone_detection.items():
            drone_dist = detection[3]
            detection_cone = detection[6]
            cvc_obst= False
            cvc_dist = False

            if drone_dist > 130:
                cvc_dist = True

            if detection_cone[0] > detection_cone[1]:
                for i in range(detection_cone[0], 181 + 1):
                    if lidar_data[i - 1] < drone_dist:
                        cvc_obst = True
                for i in range(1, detection_cone[1]):
                    if lidar_data[i - 1] < drone_dist:
                        cvc_obst = True

            else:
                for i in range(detection_cone[0], detection_cone[1] + 1):
                    if lidar_data[i - 1] < drone_dist:
                        cvc_obst = True

            if cvc_obst is True:
                CVC["CVC obst"].append(id)
            if cvc_dist is True:
                CVC["CVC dist"].append(id)

        return(CVC)


    def visual_connectvity_list(self, gap_detection_memory, drone_detection, cvc_dict):
        
        semantic_angles = compute_ray_angles(2*math.pi, 181)
        VC_list = []

        for i,frame in enumerate(gap_detection_memory):
            VC = None

            for id, detection in drone_detection.items():
                drone_index = FromAngleToRay(detection[4],semantic_angles)
                if frame[1] > frame[2]:
                    if drone_index >= frame[1] or drone_index <= frame[2]:
                        if VC is not None:
                            if detection[3] < VC[2]:
                                VC = [id, drone_index, detection[3], frame[0]]
                                if id in cvc_dict["CVC obst"]:
                                    VC.append("CVC obst")
                                else:
                                    VC.append("NCVC obst")
                                if id in cvc_dict["CVC dist"]:
                                    VC.append("CVC dist")
                                else:
                                    VC.append("NCVC dist")

                                if VC[2] < 50:
                                    VC.append("TC")
                                else:
                                    VC.append("NTC")
                        else:
                            VC = [id, drone_index, detection[3], frame[0]]
                            if id in cvc_dict["CVC obst"]:
                                VC.append("CVC obst")
                            else:
                                VC.append("NCVC obst")
                            if id in cvc_dict["CVC dist"]:
                                VC.append("CVC dist")
                            else:
                                VC.append("NCVC dist")
                            if VC[2] < 50:
                                VC.append("TC")
                            else:
                                VC.append("NTC")                           

                else:
                    if drone_index >= frame[1] and drone_index <= frame[2]:
                        if VC is not None:
                            if detection[3] < VC[2]:
                                VC = [id, drone_index, detection[3], frame[0]]
                                if id in cvc_dict["CVC obst"]:
                                    VC.append("CVC obst")
                                else:
                                    VC.append("NCVC obst")
                                if id in cvc_dict["CVC dist"]:
                                    VC.append("CVC dist")
                                else:
                                    VC.append("NCVC dist")
                                
                                if VC[2] < 50:
                                    VC.append("TC")
                                else:
                                    VC.append("NTC")    
                        else:
                            VC = [id, drone_index, detection[3], frame[0]]
                            if id in cvc_dict["CVC obst"]:
                                VC.append("CVC obst")
                            else:
                                VC.append("NCVC obst")
                            if id in cvc_dict["CVC dist"]:
                                VC.append("CVC dist")
                            else:
                                VC.append("NCVC dist")

                            if VC[2] < 50:
                                VC.append("TC")
                            else:
                                VC.append("NTC")
                
            
            if VC is not None:
                VC_list.append(VC)
        


        return VC_list




