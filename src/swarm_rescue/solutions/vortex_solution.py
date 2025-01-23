import math
import random
import numpy
from typing import Optional
import time

from statemachine import State
from statemachine import StateMachine

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle
from spg_overlay.utils.vortex_utils.PID import PID
from spg_overlay.utils.vortex_utils.ray_analyse import Analyzer
from spg_overlay.utils.vortex_utils.Potential_field import PotentialField
from spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor
from spg_overlay.entities.drone_distance_sensors import compute_ray_angles

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule
from spg_overlay.utils.vortex_utils.vortex_state_machines.procedures_state_machine import Behaviour
from spg_overlay.utils.vortex_utils.vortex_state_machines.actuators_calculator import ActuatorsComputer
from spg_overlay.utils.vortex_utils.vortex_state_machines.drone_role_state_machine import RoleState
from spg_overlay.utils.vortex_utils.vortex_state_machines.drone_situation_state_machine import Situation
from spg_overlay.utils.vortex_utils.vortex_state_machines.sensors_analyzer import SensorsAnalyzer





class MyDroneVortex(BrainModule, DroneAbstract):
    def __init__(self, signature : Optional[str] = None,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):

        DroneAbstract.__init__(self,
                               identifier= identifier,
                               misc_data= misc_data,
                               display_lidar_graph=False,
                               **kwargs)
        BrainModule.__init__(self, signature=signature)

        self.identifier = identifier
        self.network = {self.signature : self}
 

        self.sensors_analyzer = SensorsAnalyzer(signature= "sensors analyzer",identifier= self.identifier)
        self.actuators_computer = ActuatorsComputer(signature= "actuator computeur", identifier= self.identifier)
        self.role = RoleState(signature= "role", identifier= self.identifier)
        self.behavior = Behaviour(signature="behaviour", identifier= self.identifier)
        self.situation = Situation(signature= "situation", identifier= self.identifier)

        self.create_module_network(self.sensors_analyzer,
                                   self.actuators_computer,
                                   self.role,
                                   self.behavior,
                                   self.situation)

        # print(self.identifier, self.network, self.subscribers)


    def create_module_network(self, *args):
        for module in args:
            self.network[module.signature] = module
            self.create_link_with(module)

    def read_request(self):
        last_request = list(self.recieved_requests.items())[-1]
        if last_request == (self.actuators_computer.signature, "Need behaviour"):
            self.request(self.signature, self.behavior.signature, "behaviour")
        elif last_request == (self.behavior.signature, "Need situation"):
            self.request(self.signature, self.situation.signature, "situation")
        elif last_request == (self.situation.signature, "Need sensors analyze"):
            self.request(self.signature, self.sensors_analyzer.signature, "sensors analyze")
        elif last_request == (self.sensors_analyzer.signature, "Need raw data"):
            self.send(self.signature, self.sensors_analyzer.signature, ["sensor raw data", self.lidar_data(), self.semantic_data()])
    
    def read_msg(self):
        last_msg = list(self.recieved_msgs.items())[-1]
        if last_msg[1][0] == "analyzed data":
            self.send(self.signature, self.situation.signature, last_msg)
        elif last_msg[1][0] == "drone situation":
            self.send(self.signature, self.behavior.signature, last_msg)
        elif last_msg[1][0] == "drone behaviours":
            self.send(self.signature, self.actuators_computer, last_msg)
        elif last_msg[1][0] == "actuators values":
            pass

    def lidar_data(self):
        lidar_data = self.lidar_values()
        return lidar_data
    def semantic_data(self):
        semantic_data = self.semantic_values()
        return semantic_data
    def communication_data(self):
        communication_data = self.communicator.received_messages()
        return communication_data
    
     

    def define_message_for_all(self):
        pass


    def control(self):
        '''
        Cette fonction defini la request de module manager au module actuator computer.
        '''
        self.request(self.signature, self.actuators_computer.signature, "Need actuators values")

        command = self.actuators_computer.command

        return command
    

    # def transfer_msg(self, author, recipient, msg_data):
    #     '''
    #     Transfert du message au bon destinataire.
    #     '''
    #     if recipient in self.network:
    #         pass

