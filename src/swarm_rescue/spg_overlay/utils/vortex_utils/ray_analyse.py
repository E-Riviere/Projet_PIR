import math

from spg_overlay.utils.utils import normalize_angle

class Analyzer:
    def __init__(self):
 
        pass

    

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
    
    