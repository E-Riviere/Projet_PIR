from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule

class Behaviour(BrainModule, StateMachine):
    
    DroneWaitingInStock = State(initial=True)
    FirstDroneStart = State()
    LeaderLeaveTheRoot = State()
    LeaderContinuExploration = State()
    LeaderManageIntersection = State()
    CalledToEnterTheEnvironment = State()
    FollowerComeCloser = State()
    RootFollowerComeCloser = State()
    FollowerManageIntersection = State()
    BranchReconfiguation = State()

    start_first_drone = DroneWaitingInStock.to(FirstDroneStart, cond = ["first_drone_id"])

    leader_start = FirstDroneStart.to(LeaderLeaveTheRoot)

    leader_explore = (LeaderLeaveTheRoot.to(LeaderContinuExploration) |
                      LeaderManageIntersection.to(LeaderContinuExploration)
                      )
    
    agent_called = DroneWaitingInStock.to(CalledToEnterTheEnvironment, cond=["green_msg"])

    follower_come_closer = (CalledToEnterTheEnvironment.to(RootFollowerComeCloser, cond = ["msg"])|
                            RootFollowerComeCloser.to(FollowerComeCloser)|
                            FollowerManageIntersection.to(FollowerComeCloser, cond = ["msg"])|
                            FollowerComeCloser.to.itself(cond = ["msg"])|
                            BranchReconfiguation.to(FollowerComeCloser)
                            )
    
    dead_end_procedure = LeaderContinuExploration.to(BranchReconfiguation)

    intersection_procedure = (LeaderContinuExploration.to(LeaderManageIntersection)|
                    FollowerComeCloser.to(FollowerManageIntersection)|
                    RootFollowerComeCloser.to(FollowerManageIntersection)
                    )

    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier
        self.actions = self.Actions(self)

        self.behavior_set = {"name" : None, "number of actions" : None}

        self.recieved_requests = {
        "Need behavior" : None
        }

        self.recieved_msgs = {
        "drone situation" : None,
        "take root done" : None,
        "drone role" : None,
        "recieved visual com" : None,
        "com send" : None,
        "drone role set" : None,
        "situation changed to root" : None,
        "situation changed" : None,
        "move done" : None,
        "centered": None,
        "aligned" : None,
        "too close" : None
        }

        self.visual_msg = None

    def on_enter_state(self, state):

        if state.id != "DroneWaitingInStock":
            print(f"{self.identifier, state.id}")
            self.actions.update_behavior_set(self, state)
        else:
            print(f"{state.id}")
            self.actions.update_behavior_set(self, state)
        

    def read_request(self, request):
        if request == "Need behavior":
            self.request(self.signature, "Module manager", "Need role")
            self.request(self.signature, "Module manager", "Need visual com")
            self.request(self.signature, "Module manager", "Need situation")
    
    def read_msg(self, title):
        if len(self.recieved_msgs[title]) > 1:
            dico = self.recieved_msgs[title][1]

        if title == "drone situation":
            self.process_communication(self.recieved_msgs["recieved visual com"][1], dico["Visual connectivity"])
            self.procedure_determination(dico)
            self.behavior_determination()
            self.send(self.signature, "Module manager", "drone behavior", self.actions.drone_action)
        
        if title == "drone role":
            pass

        if title == "recieved visual com":
            pass


    class Actions(StateMachine):
        Stationary = State(initial=True)
        TakeRoot = State()
        LeaveRoot = State()
        FollowTheGap = State()
        RotationToTheLeftMostGap = State()
        GetCloser = State()
        Centering = State()
        ChangeRole = State()
        ChangeSituation = State()
        Sendmsg = State()

        first_agent_start_set = (Stationary.to(TakeRoot)|
                                 TakeRoot.to(ChangeRole)|
                                 ChangeRole.to(ChangeSituation)|
                                 ChangeSituation.to(Stationary)
                                 )
        
        leader_start_set = (Stationary.to(Sendmsg)|
                            Sendmsg.to(Stationary)|
                            Stationary.to(LeaveRoot)|
                            LeaveRoot.to(ChangeSituation)|
                            ChangeSituation.to(Stationary)
                            )
        
        leader_explore_branch_set = (Stationary.to(FollowTheGap)|
                                     FollowTheGap.to(Stationary)
                                     )
        
        leader_manage_intersection_set = (Stationary.to(Sendmsg)|
                                          Sendmsg.to(Centering)|
                                          Centering.to(RotationToTheLeftMostGap)|
                                          RotationToTheLeftMostGap.to(Stationary)
                                          )
        
        agent_called_set = (Stationary.to(TakeRoot)|
                            TakeRoot.to(ChangeRole)|
                            ChangeRole.to(ChangeSituation)|
                            ChangeSituation.to(Stationary)
                            )
        
        root_follower_come_closer_set = (Stationary.to(Sendmsg)|
                                         Sendmsg.to(LeaveRoot)|
                                         LeaveRoot.to(ChangeSituation)|
                                         ChangeSituation.to(GetCloser)
                                         )
        
        follower_come_closer_set = (Stationary.to(Sendmsg)|
                                    Sendmsg.to(GetCloser)|
                                    GetCloser.to(Stationary)
                                    )
        
        follower_manage_intersection_set = (Stationary.to(Centering)|
                                            Centering.to(Stationary)
                                            )

        def __init__(self, behavior):
            super().__init__()
            self.behavior = behavior
            self.drone_action = {}

        def update_behavior_set(self, drone_behavior):

            self.behavior.behavior_set["name"] = drone_behavior.id

            if drone_behavior.id == "DroneWaitingInStock":
                pass

            elif drone_behavior.id == "FirstDroneStart":
                self.behavior.behavior_set["number of actions"] = 4
                self.first_agent_start_set()
            
            elif drone_behavior.id == "LeaderLeaveTheRoot":
                self.behavior.behavior_set["number of actions"] = 5
                self.leader_start_set()

            elif drone_behavior.id == "LeaderContinuExploration":
                self.behavior.behavior_set["number of actions"] = 2
                self.leader_explore_branch_set()

            elif drone_behavior.id == "LeaderManageIntersection":
                self.behavior.behavior_set["number of actions"] = 4
                self.leader_manage_intersection_set()

            elif drone_behavior.id == "CalledToEnterTheEnvironment":
                self.behavior.behavior_set["number of actions"] = 4
                self.agent_called_set()

            elif drone_behavior.id == "FollowerComeCloser":
                self.behavior.behavior_set["number of actions"] = 3
                self.follower_come_closer_set()

            elif drone_behavior.id == "RootFollowerComeCloser":
                self.behavior.behavior_set["number of actions"] = 4
                self.root_follower_come_closer_set()

            elif drone_behavior.id == "FollowerManageIntersection":
                self.behavior.behavior_set["number of actions"] = 2
                self.follower_manage_intersection_set()
        
        def on_enter_state(self, state):
            self.behavior.behavior_set["number of actions"] -= 1
            

    def first_drone_id(self):
        return self.identifier == 0
    
    def green_msg(self):
        return self.visual_msg[0][0] == "green" and self.visual_msg[0][1] == self.identifier

    def procedure_determination(self, drone_situation):
        
        if self.behavior_set["number of acrions"] == 0:
            
            if drone_situation["Stock"]:
                self.start_first_drone()
                self.agent_called()

            




    def behavior_determination(self):

        if self.actions.drone_action["Send msg"]:
            if self.behavior_set_name["Dead end reached"]:
                self.send(self.signature, "Module manager", "send branch reconfiguration")
            if self.behavior_set_name["Leader leave the root"] or self.behavior_set_name["Root follower come closer"]:
                self.send(self.signature, "Module manager", "send more agent required") 
            if self.behavior_set_name["Leader manage intersection"]:
                self.send(self.signature, "Module manager", "send come closer")     



    def process_communication(self, communication, vc_list):

        if len(communication) > 0:
            coms = []
            for com in communication:
                coms.append(com[1])

            if  vc_list[0] is not None:
                id_vc = [int(vc[0]) for vc in vc_list[1]]

                for com in coms:
                    if com["id"] in id_vc:
                        if len(com["visual msgs"])>0 and "purple" in com["visual msgs"]:
                            self.visual_msg = "purple"
                            print(self.identifier, self.visual_msg)


            if self.behavior_set_name["Drone waiting in stock"]:
                for com in coms:
                    if len(com["visual msgs"])>0 and com["visual msgs"][0][1] == self.identifier:
                        self.visual_msg = com["visual msgs"][0][0]
                        print(self.identifier, self.visual_msg)

            if (self.behavior_set_name["Leader leave the root"]
                and len(list(self.behavior_set.values())) > 0
                and self.drone_behaviors["Leave root"]):
                for com in coms:
                    if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
                        del self.behavior_set["Leave root"]
                        self.behavior_determination()

            if (self.behavior_set_name["Root follower come closer"]
                and len(list(self.behavior_set.values())) > 0
                and self.drone_behaviors["Leave root"]):
                for com in coms:
                    if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
                        del self.behavior_set["Leave root"]
                        self.behavior_determination()
                        self.send(self.signature, "Module manager", "change situation from root")
            

                        







        
    
        
    


    


