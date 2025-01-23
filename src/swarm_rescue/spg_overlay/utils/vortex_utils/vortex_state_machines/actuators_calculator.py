from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule

class ActuatorsComputer(BrainModule):
    # Stationary = State(initial=True)
    # # FollowTheGap = State()
    # # LeaveRoot = State()
    # TakeRoot = State()
    # # GetCloser = State()
    # # TurnAround = State()
    # # Centering = State()
    # # FollowTheLeftMostGap = State()
    # # SendMessage = State()
    # # ChangeRole = State()

    # agent_entrance = Stationary.to(TakeRoot) 
    # agent_stop_at_root = TakeRoot.to(Stationary)


    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier

        self.command = {
        "forward" : 0.0,
        "lateral" : 0.0,
        "rotation" : 0.0
        }

        self.behaviour_memory = None

    def read_request(self):
        last_request = list(self.recieved_requests.items())[-1]
        if last_request[1] == "Need actuators values":
            if self.behaviour_memory == None:
                self.request(self.signature, "module manager", "Need behavior")
    
    def read_msg(self):
        last_msg = list(self.recieved_msgs.items())[-1]
        if last_msg[1][0] == "drone behaviours":
            self.computer(last_msg)
            self.send(self.signature, "module manager", ["actuators values", self.command])
                     
    
    def computer(self, last_msg):
        if self.behaviour_memory == "Stationary":
             self.stationary_command()
        if self.behaviour_memory == "Take root":
            self.take_root_control_command()
        pass
    

    def stationary_command(self):
        self.command["forward"] = 0.0
        self.command["lateral"] = 0.0
        self.command["rotation"] = 0.0


    def take_root_control_command(self, gps_pose, root_pose):
        eps = 10**(0)
        if gps_pose[1] - root_pose[1] > eps:
            self.forward = 0.0
            self.lateral = -0.1
            self.rotation = 0.0

        elif gps_pose[1] - root_pose[1] < -eps:
            self.forward = 0.0
            self.lateral = 0.1
            self.rotation = 0.0

        elif gps_pose[0] - root_pose[0] < -eps:
            self.forward = 0.1
            self.lateral = 0.0
            self.rotation = 0.0    

        elif gps_pose[0] - root_pose[0] > eps:
            self.forward = -0.1
            self.lateral = 0.0
            self.rotation = 0.0
        else:
            pass    



    # def brain_tickle_action_for_control(self, gps, root):
    #     if self.current_state.id == "TakeRoot":
    #         self.take_root_control_command(gps, root)
    #     if self.current_state.id == "Stationary":
    #         self.stationary_command()





    
    # def on_enter_Stationary(self):
    #     print("stationary")
    # def on_enter_TakeRoot(self):
    #     print(self.identifier, "take root")

        




    

