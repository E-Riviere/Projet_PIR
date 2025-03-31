from statemachine import State
from statemachine import StateMachine


class drone_waiting_stock_set(StateMachine):
    Idle =State(initial=True)

    drorne_waiting = Idle.to.itself(internal=True)

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : None, "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Idle":
            self.drone_action["action"] = state.id


class first_agent_start_set(StateMachine):
    Idle = State(initial=True)
    TakeRoot = State()
    ChangeRole = State()
    ChangeSituation = State()
    Stationary = State()

    first_agent_start = (Idle.to(TakeRoot)|
                        TakeRoot.to(ChangeRole)|
                        ChangeRole.to(ChangeSituation)|
                        ChangeSituation.to(Stationary)|
                        Stationary.to.itself()
                        )
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : None, "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Idle":
            self.drone_action["action"] = state.id

    def on_enter_ChangeRole(self):
        self.behavior.send(self.behavior.signature, "Module manager", "set first drone role")

    def on_enter_ChangeSituation(self):
        self.behavior.send(self.behavior.signature, "Module manager", "change situation to root")


class leader_start_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    LeaveRoot = State()
    ChangeSituation = State()

    leader_start = (Stationary.to(Sendmsg)|
                    Sendmsg.to(LeaveRoot)|
                    LeaveRoot.to(ChangeSituation)|
                    ChangeSituation.to(Stationary)
                    )
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_ChangeSituation(self):
        self.behavior.send(self.behavior.signature, "Module manager", "change situation from root")

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send more agent required")


class leader_explore_branch_set(StateMachine):
    Stationary = State(initial=True)
    FollowTheGap = State()

    leader_explore_branch = (Stationary.to(FollowTheGap)|
                            FollowTheGap.to(Stationary)
                            )
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_FollowTheGap(self):
        if isinstance(self.behavior.recieved_msgs["drone situation"][1]["Intersection"], list):
            self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1
        elif self.behavior.recieved_msgs["drone situation"][1]["Corridor"]:
            self.drone_action["gap sel id"] = 1


class leader_manage_intersection_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    Centering = State()
    RotationToTheLeftMostGap = State()
    FollowTheGap = State()

    leader_manage_intersection = (Stationary.to(Sendmsg)|
                                        Sendmsg.to(Centering)|
                                        Centering.to(RotationToTheLeftMostGap)|
                                        RotationToTheLeftMostGap.to(FollowTheGap)|
                                        FollowTheGap.to(Stationary)
                                        )
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_FollowTheGap(self):
        if isinstance(self.behavior.recieved_msgs["drone situation"][1]["Intersection"], list):
            self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1
        elif self.behavior.recieved_msgs["drone situation"][1]["Corridor"]:
            self.drone_action["gap sel id"] = 1

    def on_enter_RotationToTheLeftMostGap(self):
        self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send come closer") 


class leader_waiting_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()

    leader_waiting = (Stationary.to(Sendmsg)|
                      Sendmsg.to(Stationary))

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send come closer")
    

class agent_called_set(StateMachine):
    Idle = State(initial=True)
    TakeRoot = State()
    ChangeRole = State()
    ChangeSituation = State()
    Stationary = State()

    agent_called = (Idle.to(TakeRoot)|
                    TakeRoot.to(ChangeRole)|
                    ChangeRole.to(ChangeSituation)|
                    ChangeSituation.to(Stationary)  
                    )
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : None, "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Idle":
            self.drone_action["action"] = state.id

    def on_enter_ChangeRole(self):
        self.behavior.send(self.behavior.signature, "Module manager", "set drone role")

    def on_enter_ChangeSituation(self):
        self.behavior.send(self.behavior.signature, "Module manager", "change situation to root")


class root_follower_come_closer_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    LeaveRoot = State()
    ChangeSituation = State()
    GetCloser = State()

    root_follower_come_closer = (Stationary.to(Sendmsg)|
                                        Sendmsg.to(LeaveRoot)|
                                        LeaveRoot.to(ChangeSituation)|
                                        ChangeSituation.to(GetCloser)|
                                        GetCloser.to(Stationary)
                                    )

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_ChangeSituation(self):
        self.behavior.send(self.behavior.signature, "Module manager", "change situation from root")

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send more agent required")


class follower_come_closer_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    GetCloser = State()
    RotationToTheLeftMostGap = State()

    follower_come_closer = (Stationary.to(Sendmsg, unless ="flagcond")|
                            Sendmsg.to(Stationary)|
                            Stationary.to(GetCloser, unless = "inter")|
                            Stationary.to.itself(on = "stat", unless = "tooclose")|
                            GetCloser.to(Stationary)|
                            Stationary.to(RotationToTheLeftMostGap, cond = "tooclose")|
                            RotationToTheLeftMostGap.to(GetCloser)|
                            GetCloser.to(Stationary))
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.flag = False
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id
    
    def on_enter_Sendmsg(self):
        self.flag = True
        self.behavior.send(self.behavior.signature, "Module manager", "send come closer")
    
    def on_enter_RotationToTheLeftMostGap(self):
        self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1
    
    def tooclose(self):
        return self.behavior.recieved_msgs["drone situation"][1]["Visual connectivity"][1][0][6] == "TC"
    
    def inter(self):
        return isinstance(self.behavior.recieved_msgs["drone situation"][1]["Intersection"],list)
    
    def stat(self, state):
        self.drone_action["action"] = state.id
    
    def visualmsg(self):
        return self.behavior.visual_msg == "purple"

    def flagcond(self):
        return self.flag


class follower_manage_intersection_set(StateMachine):
    Stationary = State(initial=True)
    Centering = State()

    follower_manage_intersection = (Stationary.to(Centering, unless="wait")|
                                    Centering.to(Stationary)|
                                    Stationary.to.itself(on = "stat"))

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}
        self.flag = False
    
    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id
    
    def on_enter_Centering(self):
        self.flag = True 
    
    def wait(self):
        return self.flag
    
    def stat(self, state):
        self.drone_action["action"] = state.id

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send come closer")


class interruption_set(StateMachine):
    Stationary = State(initial=True)

    interruption = Stationary.to.itself(internal = True)

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id


class branch_reconfiguration_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    ChangeRole = State()
    TurnAround = State()
    

    branch_reconfiguration = (Stationary.to(ChangeRole)|
                              ChangeRole.to(TurnAround)|
                              TurnAround.to(Sendmsg)|
                              Sendmsg.to(Stationary))
                            
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_ChangeRole(self):
        self.behavior.send(self.behavior.signature, "Module manager", "set drone role")

    def on_enter_TurnAround(self):
        self.drone_action["gap sel id"] = 0
    
    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send branch reconfiguration")

    






