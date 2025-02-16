import math
import numpy

from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule
from spg_overlay.utils.vortex_utils.PID import PID
from spg_overlay.utils.vortex_utils.Potential_field import PotentialField

class ActuatorsComputer(BrainModule):

    def __init__(self,
                 signature,
                 identifier,
                 lidar_angle
                 ):
        
        super().__init__(signature)
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

        self.leave_root_pid = PID(Kp=0.1, Ki=0.02, Kd=0.02, output_limits=(-0.1,0.1))
        self.leave_root_pid.setpoint = 50
        self.follower_pid_1 = PID(Kp=0.1, Ki=0.02, Kd=0.02, output_limits=(-0.1,0.1))
        self.follower_pid_1.setpoint = 125
        self.follower_pid_2 = PID(Kp=0.1, Ki=0.02, Kd=0.02, output_limits=(-0.1,0.1))
        self.follower_pid_2.setpoint = 50


    def read_request(self, request):
        if request == "Need actuators values":
            self.request(self.signature, "Module manager", "Need collision detection")
            self.request(self.signature, "Module manager", "Need behavior")
    
    def read_msg(self, title):
        dico = self.recieved_msgs[title][1]
        # print(last_msg)
        if title == "drone behavior":

            self.command_selector(dico)
            potential_field_command = self.potential_field.TotalRepulsivePotentialFieldVector(self.recieved_msgs["collision detection"][1], self.lidar_angle)
            if potential_field_command is not None:
                (self.command["forward"], self.command["lateral"]) = potential_field_command
            self.send(self.signature, "Module manager", "actuators values", self.command)

        elif title == "gps pos":
            pass

        elif title == "drone detection":
            pass

        elif title == "gap dir":
            pass

        elif title == "neg gap dist":
            pass

        elif title == "collision detection":
            pass
                     
    
    def command_selector(self, drone_behaviors):

        if drone_behaviors["Take root"]:
            self.request(self.signature, "Module manager", "Need gps pos")
            gps_pos = self.recieved_msgs["gps pos"][1]
            self.take_root_control_command(gps_pos, (-250, 15))
        
        if drone_behaviors["Stationary"]:
            self.stationary_command()
        
        if drone_behaviors["Leave root"]:
            self.request(self.signature, "Module manager", "Need drone detection")
            if len(self.recieved_msgs["drone detection"][1]):
                drone_dist = self.recieved_msgs["drone detection"][1][0][2]
                self.leave_root_control_command(drone_dist)
            else:
                pass
        
        if  isinstance(drone_behaviors["Follow the gap"], list) and drone_behaviors["Follow the gap"][0] is True:
            self.request(self.signature, "Module manager", "Need positive gap directions")
            gap_dir = self.recieved_msgs["gap dir"][1]
            self.FollowTheGap(gap_dir, drone_behaviors["Follow the gap"][1])
        
        if drone_behaviors["Centering"]:
            self.request(self.signature, "Module manager", "Need negative gap distance")
            neg_gap_dist = self.recieved_msgs["neg gap dist"][1]
            self.CenterInIntersection(neg_gap_dist)

        if isinstance(drone_behaviors["Rotation to the left most gap"], list) and drone_behaviors["Rotation to the left most gap"][0] is True:
            self.request(self.signature, "Module manager", "Need positive gap directions")
            gap_dir = self.recieved_msgs["gap dir"][1]
            self.AlignWithTheGap(gap_dir, drone_behaviors["Rotation to the left most gap"][1])
        
        if drone_behaviors["Get closer"]:
            self.request(self.signature, "Module manager", "Need drone detection")
            if len(self.recieved_msgs["drone detection"][1]):
                if self.recieved_msgs["drone detection"][1][-1][5] == "NTC":
                    drone_dist = self.recieved_msgs["drone detection"][1][0][2]
                    self.follower_control_command(drone_dist, 1)
                
                else:
                    drone_dist = self.recieved_msgs["drone detection"][1][-1][2]
                    self.follower_control_command(drone_dist, 2)


    def stationary_command(self):

        self.command["forward"] = 0.0
        self.command["lateral"] = 0.0
        self.command["rotation"] = 0.0
        self.send(self.signature, "Module manager", "stationary")


    def take_root_control_command(self, gps_pose, root_pose):
        # print("b")
        eps = 10**(0)
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
        else:
            self.send(self.signature, "Module manager", "take root done")


    def leave_root_control_command(self, drone_dist):
        if drone_dist < 50:
            if drone_dist - self.leave_root_pid.setpoint > 0:
                self.command["forward"] = self.leave_root_pid(drone_dist)
                self.command["lateral"] = 0.0
                self.command["rotation"] = 0.0

            elif drone_dist - self.leave_root_pid.setpoint < 0:
                self.command["forward"] = self.leave_root_pid(drone_dist)
                self.command["lateral"] = 0.0
                self.command["rotation"] = 0.0
        else:
            self.command["forward"] = 0.0
            self.command["lateral"] = 0.0
            self.command["rotation"] = 0.0           


    def FollowTheGap(self, gap_analysis, gap_sel):
        
        self.command["forward"] = 0.1

        if gap_analysis[gap_sel] > 0:
            self.command["rotation"] = (abs(gap_analysis[gap_sel])/math.pi) 
            self.command["lateral"] = 0.1

        if gap_analysis[gap_sel] < 0:
            self.command["rotation"] = -(abs(gap_analysis[gap_sel])/math.pi)
            self.command["lateral"] = -0.1

        else:
            pass
        self.send(self.signature, "Module manager", "move done")



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
            self.send(self.signature, "Module manager", "centered")


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
            self.send(self.signature, "Module manager", "aligned")

    
    def follower_control_command(self, drone_dist, pid):
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
                

    
    

        




    

