from spg_overlay.utils.vortex_utils.Potential_field import PotentialField 
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.vortex_utils.vortex_state_machines.sensors_analyzer import SensorsAnalyzer
from spg_overlay.utils.vortex_utils.vortex_state_machines.actuators_calculator import ActuatorsComputer
from spg_overlay.utils.misc_data import MiscData

from typing import Optional

import random

import numpy as np

import statistics
import socket

class MyDronePIR(DroneAbstract):

    sensors_analyzer = None
    actuators_computer = None
    goal_data_analyze = None
    choosed_gap = None
    goal_compass = None
    count = None
    count_send = 0
    state = "waiting connection"
    
    socket = int(input("socket :"))
    client_socket = None

    def __init__(self,
                identifier: Optional[int] = None,
                misc_data: Optional[MiscData] = None,
                **kwargs):
        super().__init__(identifier=identifier,
                        misc_data=misc_data,
                        **kwargs)
        
        self.actuators_computer = ActuatorsComputer(identifier,self.lidar_rays_angles())
        self.sensors_analyzer = SensorsAnalyzer(identifier)
        self.sensors_analyzer.disable = False
        self.close_socket_com = False
        # self.connect()
        # print(self.client_socket)

    def control(self):
        x,y = self.true_position()
        print("coo",x,y)
        print("coucou", self.client_socket)
        if self.client_socket == None:
            self.connect()
            print("a",self.client_socket)
        else:
            self.count_send += 1
            if self.count_send%10 == 0:
                m = f"{str(x)} {str(y)};"
                self.client_socket.sendall(m.encode())
        if self.state == "waiting connection":
            mes = self.client_socket.recv(1024).decode()
            if mes == "Connected":
                print("Starting")
                self.state = "take root"
        lidar_values = self.lidar_values()
        self.returning_last_center = False
        if self.state == "take root":
            self.actuators_computer.take_root_control_command(self.gps_values(),(-200,0))
        
            if (x + 200)**2 + (y)**2 < 50:
                self.state = "follow the gap"
                print(self.state)
                self.sensors_analyzer.disable = False
        elif self.state == "follow the gap": 
            print(self.compass_values())
            print(self.sensors_analyzer.analyzed_data['positive gap number'])
            if self.sensors_analyzer.analyzed_data['positive gap number'] > 0:
                if self.sensors_analyzer.analyzed_data['positive gap number'] < 3 :
                    if self.sensors_analyzer.analyzed_data['positive gap number'] == 1 and not self.returning_last_center:
                        self.state = "align with gap"
                        print(self.state)

                        print(self.sensors_analyzer.analyzed_data['positive gap direction'])

                        print(self.compass_values())
                        print()
                        pos_gap_dir = self.sensors_analyzer.analyzed_data['positive gap direction']
                        self.choosed_gap = 0
                        self.goal_compass = pos_gap_dir[self.choosed_gap]
                        self.returning_last_center = True
                        self.init_compass_value = self.compass_values()
                    else:
                        self.goal_data_analyze = self.sensors_analyzer.analyzed_data['positive gap direction']
                        self.actuators_computer.FollowTheGap(self.goal_data_analyze,-1)
                    
                else:
                    self.state = "center in intersection"
                    print(self.state)
                    
                
            
        elif self.state == "center in intersection":
            if self.goal_data_analyze:
                self.actuators_computer.CenterInIntersection(self.sensors_analyzer.analyzed_data['negative gap dist ray'])
                min_dist = []
                self.returning_last_center = False
                for obst in self.sensors_analyzer.analyzed_data['negative gap dist ray']:
                    min_dist.append(obst[1])
                
                if statistics.variance(min_dist) < 5 and np.linalg.norm(self.measured_velocity()) < 0.1:
                    self.state = "align with gap"
                    print(self.state)
                    # self.choosed_gap = random.randint(0,self.sensors_analyzer.analyzed_data['positive gap number'] - 1)
                    self.choosed_gap = -1
                    pos_gap_dir = self.sensors_analyzer.analyzed_data['positive gap direction']
                    self.goal_compass = pos_gap_dir[self.choosed_gap]
                    self.init_compass_value = self.compass_values()

        elif self.state == "align with gap":
            
            
            #self.choosed_gap = 0
            #self.goal_compass = pos_gap_dir[self.choosed_gap]
            #print(self.choosed_gap )
                # gaps = self.sensors_analyzer.analyzed_data['positive gap direction'][choosed_gap]
            self.actuators_computer.AlignWithTheGap(self.sensors_analyzer.analyzed_data['positive gap direction'],self.choosed_gap )
            print(abs(abs(self.compass_values() - self.init_compass_value) - abs(self.goal_compass)))
            print(self.init_compass_value)
            print(self.compass_values())
            print(self.goal_compass)
            print(self.measured_angular_velocity())
            if abs(abs(self.compass_values() - self.init_compass_value) - abs(self.goal_compass)) < 0.1 and self.measured_angular_velocity() < 0.1 :
                self.state = "enter the gap"
                print(self.state)
                self.count = 0

        # if (x + 200)**2 + (y)**2 < 50:
        #     self.initialized = True
        #     self.sensors_analyzer.disable = False
        self.sensors_analyzer.analyze(self.lidar_values(),self.lidar_rays_angles(),self.semantic_values())
        command = self.actuators_computer.command

        if self.state == "enter the gap":
            self.count += 1
            command = {
                "forward" : 0.5,
                "lateral" : 0.0,
                "rotation" : 0.0
            }
            if self.count > 20:
                self.state = "follow the gap"
                print(self.state)

        
        return command
    
    def define_message_for_all(self):
        return super().define_message_for_all()

    def connect(self):
        self.close_socket_com
        HOST = "localhost"
        PORT = self.socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.client_socket.setblocking(False)
        try :
            self.client_socket.connect((HOST, PORT))
            print(self.client_socket)
        except OSError as e:
            if e.errno !=115:
                raise
            print("Error connecting to server:", e)
            self.client_socket.close()
            self.client_socket = None
            return
        else:
            print(self.client_socket)
            print("Connected to server")
            print(self.client_socket)
            