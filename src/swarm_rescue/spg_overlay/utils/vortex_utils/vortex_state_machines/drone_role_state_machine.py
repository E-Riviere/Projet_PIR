from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule

class RoleState(BrainModule):
    # Idle = State(initial=True)
    # Leader = State()
    # Follower = State()

    # First_drone = Idle.to(Leader)
    # Called_to_enter_env = Idle.to(Follower)
    # Take_lead = Follower.to(Leader)
    # Give_lead = Leader.to(Follower)



    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier

        self.recieved_requests = {
        }

        self.recieved_msgs = {
        "change role" : None,
        "set first drone role" : None,
        "set drone role" : None
        }

        self.actual_role = {
        "Leader" : False,
        "Root Follower" : False,
        "Reconfiguration Follower" : False
        }

    def read_request(self, request):
        pass
    
    def read_msg(self, title):
        if len(self.recieved_msgs[title]) > 1:
            dico = self.recieved_msgs[title][1]

        if title == "set first drone role":
            self.actual_role["Leader"] = True
            self.actual_role["Root Follower"] = False
            self.actual_role["Reconfiguration Follower"] = False
            self.send(self.signature, "Module manager", "drone role set")

        if title == "set drone role":
            self.actual_role["Leader"] = False
            self.actual_role["Root Follower"] = True
            self.actual_role["Reconfiguration Follower"] = False
            self.send(self.signature, "Module manager", "drone role set")
        
        if title == "set reconfiguration role":
            self.actual_role["Leader"] = False
            self.actual_role["Root Follower"] = False
            self.actual_role["Reconfiguration Follower"] = True
            self.send(self.signature, "Module manager", "drone role set")

        if title == "set new leader role":
            self.actual_role["Leader"] = True
            self.actual_role["Root Follower"] = False
            self.actual_role["Reconfiguration Follower"] = False
            self.send(self.signature, "Module manager", "drone role set")



