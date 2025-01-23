from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule

class Behaviour(BrainModule):

    # WaitingInStock = State(initial=True)
    # LeaderStart = State(value = 2)
    # # CalledToEnterTheEnvironment = State()
    # # NeedSomeoneCloser = State()
    # # LeaveTheRoot = State()
    # # ExplorationPossible = State()
    # # PositionReplacement = State()
    # # BeingReplacedAtPose = State()
    # # DeadEndReached = State()
    # # BranchReconfiguration = State()
    # # LoopDetected = State ()

    # Leader_start = WaitingInStock.to(LeaderStart, on = "actions_leader_start") | LeaderStart.to.itself(on = "actions_leader_start") | LeaderStart.to.itself(on = "actions_leader_start")

    
    
    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier
        self.actions_per_procedure = None

        self.drone_behaviours = {
        
        }


    def read_request(self):
        last_request = list(self.recieved_requests.items())[-1]
        if last_request[1] == "behaviour":
            self.request(self.signature, "module manager", "Need situation")
    
    def read_msg(self):
        last_msg = list(self.recieved_msgs.items())[-1]
        if last_msg[1][0] == "drone situation":
            self.behaviour_determination(last_msg)
            self.send(self.signature, "module manager", ["drone behaviours", self.drone_behaviours])
    
    def behaviour_determination(self, drone_situation):
        pass
















    # def brain_tickle_procedure_for_situation(self, situation_name):
    #     if self.current_state.id == "WaitingInStock" and situation_name == "in stock" and self.identifier == 0:
    #         self.Leader_start()
    # def brain_tickle_procedure_for_action(self):
    #     if self.current_state.id == "LeaderStart":
    #         self.Leader_start()

    # def actions_per_procedure_setter(self):
    #     self.actions_per_procedure = self.LeaderStart.value

    # def actions_leader_start(self):
    #     if self.actions_per_procedure == None:
    #         self.actions_per_procedure_setter()
    #     if self.actions_per_procedure == 2:
    #         self.brain.procedure_tickle_brain_to_action("take the root")
    #         self.brain.procedure_tickle_brain_to_role("change role to leader")
    #         self.actions_per_procedure += -1

    #     elif self.actions_per_procedure == 1:
    #         self.brain.procedure_tickle_brain_to_action("stop at root")
    #         self.brain.procedure_tickle_brain_to_situation("change situation to root")
    #         self.actions_per_procedure += -1

    
    # def before_Leader_start(self):
        print(self.identifier, "leader start")

        
    
        
    


    


