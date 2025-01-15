import math
import numpy


from spg_overlay.utils.utils import normalize_angle
from spg_overlay.utils.vortex_utils.ray_analyse import Analyzer

class PotentialField():
    def __init__(self):
          self.analyzer = Analyzer()
    
    def UnitRepulsivePotentialFieldVector(self, Obst, LIDARProcess):
            alpha = 0.001
            k_rep = 0.1
            U_rep = (k_rep * numpy.exp(-alpha * Obst[1]))
            teta = normalize_angle(self.analyzer.FromRayToAngle(Obst[0], LIDARProcess) + math.pi)
            return ((U_rep * math.cos(teta), U_rep * math.sin(teta)))
        
    def TotalRepulsivePotentialFieldVector(self, collision_detection, LIDARProcess):
        (U_tot_x, U_tot_y) = (0.0, 0.0)
        for Obst in collision_detection:
            (U_rep_x, U_rep_y) = self.UnitRepulsivePotentialFieldVector(Obst, LIDARProcess)
            U_tot_x += U_rep_x
            U_tot_y += U_rep_y
        norm = math.sqrt(U_tot_x**2 + U_tot_y**2)
        return ((U_tot_x / norm, U_tot_y / norm))