import math
import numpy

from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.PID import PID
from spg_overlay.utils.vortex_utils.Potential_field import PotentialField

class ActuatorsComputer():

    def __init__(self,
                 identifier,
                 lidar_angle
                 ):
        
        self.identifier = identifier
        self.lidar_angle = lidar_angle

        self.command = {
        "forward" : 0.0,
        "lateral" : 0.0,
        "rotation" : 0.0
        }

        self.recieved_requests = {
        "Need actuators values" : None
        }

        self.recieved_msgs = {
        "gps pos" : None,
        "drone detection" : None,
        "drone behavior" : None,
        "gap dir" : None,
        "neg gap dist" : None,
        "collision detection" : None
        }

        self.potential_field = PotentialField()

        self.follower_pid_1 = PID(Kp=0.1, Ki=0.02, Kd=0.02, output_limits=(-0.1,0.1))
        self.follower_pid_1.setpoint = 125
        self.follower_pid_2 = PID(Kp=0.1, Ki=0.02, Kd=0.02, output_limits=(-0.1,0.1))
        self.follower_pid_2.setpoint = 50

  


    def stationary_command(self):

        self.command["forward"] = 0.0
        self.command["lateral"] = 0.0
        self.command["rotation"] = 0.0


    def take_root_control_command(self, gps_pose, root_pose):
        eps = 10**(0)   
        print("gps pose", gps_pose)
        print("root pose", root_pose)
        if gps_pose[1] - root_pose[1] > eps:
            self.command["forward"] = 0.0
            self.command["lateral"] = -0.1
            self.command["rotation"] = 0.0

        elif gps_pose[1] - root_pose[1] < -eps:
            self.command["forward"] = 0.0
            self.command["lateral"] = 0.1
            self.command["rotation"] = 0.0

        elif gps_pose[0] - root_pose[0] < -eps:
            self.command["forward"] = 0.1
            self.command["lateral"] = 0.0
            self.command["rotation"] = 0.0    

        elif gps_pose[0] - root_pose[0] > eps:
            self.command["forward"] = -0.1
            self.command["lateral"] = 0.0
            self.command["rotation"] = 0.0         


    def FollowTheGap(self, gap_analysis, gap_sel):
        
        self.command["forward"] = 0.2

        if gap_analysis[gap_sel] > 0:
            self.command["rotation"] = (abs(gap_analysis[gap_sel])/math.pi) 
            self.command["lateral"] = 0.1

        if gap_analysis[gap_sel] < 0:
            self.command["rotation"] = -(abs(gap_analysis[gap_sel])/math.pi)
            self.command["lateral"] = -0.1

        else:
            pass



    def CenterInIntersection(self, Obst_dist_ray):
        cond = False
        self.command["rotation"] = 0.0
        if Obst_dist_ray[-1][1] - Obst_dist_ray[1][1] < -0.5:
            self.command["forward"] = 0.05
            cond = True
        elif Obst_dist_ray[-1][1] - Obst_dist_ray[1][1] > 0.5:
            self.command["forward"] = -0.05
            cond = True
        if Obst_dist_ray[0][1] - Obst_dist_ray[-1][1] < -0.5:
            self.command["lateral"] = 0.05
            cond = True
        elif Obst_dist_ray[0][1] - Obst_dist_ray[-1][1] > 0.5:
            self.command["lateral"] = -0.05
            cond = True
        
        if not cond:
            self.command["forward"] = 0.0
            self.command["lateral"] = 0.0
            self.command["rotation"] = 0.0


    def AlignWithTheGap(self, gap_analysis, gap_selected):
        direction = gap_analysis[gap_selected]

        self.command["forward"] = 0.0
        self.command["lateral"] = 0.0
        self.command["rotation"] = 0.0

        if direction > 0.05:
            self.command["rotation"] = 0.2

        elif direction < -0.05:
            self.command["rotation"] = - 0.2
        
        else:
            self.command["rotation"] = 0.0
            

    
    def follower_control_command(self, drone_dist, pid, ray):
        self.command["forward"] = 0.0
        self.command["lateral"] = 0.0
        self.command["rotation"] = 0.0

        if pid==1:
            self.command["forward"] = self.follower_pid_1(drone_dist)
            self.command["lateral"] = 0.0
            self.command["rotation"] = 0.0

        elif pid==2:
            self.command["forward"] = -self.follower_pid_2(drone_dist)
            self.command["lateral"] = 0.0
            self.command["rotation"] = 0.0
 

                

    
    

        




    

