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

class MyDronePIRv2(DroneAbstract):

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
        self.choosed_gap = None
        self.goal_compass = None
        self.count = None
        self.count_send = 0
        self.previous_state = ""
        self.state = "init"
        self.is_controlled = False
        self.ask_controlled = input("is controlled ? (y/n) :")
        if self.ask_controlled == "y":
            self.is_controlled = True
        else:
            self.is_controlled = False
        if self.is_controlled:
            self.socket = int(input("socket :"))
            self.state = "waiting connection"
        self.client_socket = None
        self.centered = False
        self.aligned = False
        self.aligning = False

    def control(self):
        x,y = self.true_position()
        print("coo",x,y)

        if self.is_controlled:
            print("coucou", self.client_socket)
            if self.client_socket == None:
                self.connect()
                print("a",self.client_socket)
            else:
                self.count_send += 1
                if self.count_send%20 == 0:
                    m = f"{str(x)} {str(y)};"
                    self.client_socket.sendall(m.encode())
            if self.state == "waiting connection":
                mes = self.client_socket.recv(1024).decode()
                if mes == "Connected":
                    print("Starting")
                    self.previous_state = self.state
                    self.state = "init"

        self.choosed_gap = -1
        print(self.state)
        self.sensors_analyzer.analyze(self.lidar_values(),self.lidar_rays_angles(),self.semantic_values())
        if self.state != "init":
            gap_detect = self.sensors_analyzer.analyzed_data['positive gap direction']
            print(gap_detect)
            if len(gap_detect) == 1:
                self.previous_state = self.state
                self.state = "deadend"
            elif len(gap_detect) == 2:
                self.previous_state = self.state
                self.state = "gap"
            elif len(gap_detect) >=3 :
                self.previous_state = self.state
                self.state = "intersection"

        if self.state == "init":
            print(self.state)
            if self.previous_state != self.state:
                self.centered = False
                self.aligned = False
            progress = self.actuators_computer.take_root_control_command(self.gps_values(),(-200,0))
            if progress == "reached root":
                self.previous_state = self.state
                self.state = "waiting for state"
        
        if self.state == "gap":
            print(self.state)
            self.actuators_computer.FollowTheGap(gap_detect,-1)
            
        if self.state == "intersection":
            print(self.state)
            self.goal_data_analyze = self.sensors_analyzer.analyzed_data['positive gap direction']
            if self.previous_state != self.state:
                self.centered = False
                self.aligned = False
            if not self.centered:
                self.centered = self.actuators_computer.CenterInIntersection(self.sensors_analyzer.analyzed_data['negative gap dist ray'])
            if self.centered and not self.aligned:
                self.aligned = self.actuators_computer.AlignWithTheGap(self.sensors_analyzer.analyzed_data['positive gap direction'],self.choosed_gap)
            if self.centered and self.aligned:
                self.actuators_computer.FollowTheGap(self.goal_data_analyze,self.choosed_gap)
            
        
        if self.state == "deadend"  :
            print(self.previous_state)
            print(self.state)
            print(self.centered)
            print(self.aligned)
            print(self.aligning)
            self.goal_data_analyze = self.sensors_analyzer.analyzed_data['positive gap direction']
            print(self.goal_data_analyze)
            print(self.choosed_gap)
            if self.previous_state != self.state:
                self.aligning = True
                self.aligned = False
                self.actuators_computer.stationary_command()
            if self.aligned:
                self.aligning = False
                self.actuators_computer.FollowTheGap(self.goal_data_analyze,self.choosed_gap)
            if self.aligning:
                self.aligned = self.actuators_computer.AlignWithTheGap(self.sensors_analyzer.analyzed_data['positive gap direction'],self.choosed_gap)
            
            
        command = self.actuators_computer.command
        return command
    
    def define_message_for_all(self):
        return super().define_message_for_all()

    def connect(self):
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
            