from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule

class RoleState(BrainModule):
    Idle = State(initial=True)
    Leader = State()
    Follower = State()

    First_drone = Idle.to(Leader)
    Called_to_enter_env = Idle.to(Follower)
    Take_lead = Follower.to(Leader)
    Give_lead = Leader.to(Follower)



    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier

    def read_request(self):
        pass
    
    def read_msg(self):
        pass

    
    def on_enter_Leader(self):
        print(self.identifier, "leader")   

