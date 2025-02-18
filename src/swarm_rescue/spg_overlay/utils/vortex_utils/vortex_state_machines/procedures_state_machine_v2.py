from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule

class Behaviour(BrainModule, StateMachine):
    
    DroneWaitingInStock = State(initial=True)
    FirstDroneStart = State()
    LeaderLeaveTheRoot = State()
    LeaderContinuExploration = State()
    LeaderManageIntersection = State()
    LeaderWaiting = State()
    CalledToEnterTheEnvironment = State()
    FollowerComeCloser = State()
    RootFollowerComeCloser = State()
    FollowerManageIntersection = State()
    BranchReconfiguration = State()
    BehaviorInterruption = State()

    start_first_drone = DroneWaitingInStock.to(FirstDroneStart, cond = ["first_drone_id"])

    leader_start = FirstDroneStart.to(LeaderLeaveTheRoot)

    leader_explore = (LeaderLeaveTheRoot.to(LeaderContinuExploration) |
                      LeaderManageIntersection.to(LeaderContinuExploration)|
                      LeaderWaiting.to(LeaderContinuExploration)|
                      BehaviorInterruption.to(LeaderContinuExploration)
                      )
    
    agent_called = DroneWaitingInStock.to(CalledToEnterTheEnvironment)

    follower_come_closer = (CalledToEnterTheEnvironment.to(RootFollowerComeCloser)|
                            RootFollowerComeCloser.to(FollowerComeCloser)|
                            FollowerManageIntersection.to(FollowerComeCloser)|
                            FollowerComeCloser.to.itself()|
                            BranchReconfiguration.to(FollowerComeCloser)
                            )
    
    leader_wait_procedure = LeaderContinuExploration.to(LeaderWaiting)

    dead_end_procedure = LeaderContinuExploration.to(BranchReconfiguration)

    intersection_procedure = (LeaderContinuExploration.to(LeaderManageIntersection)|
                    FollowerComeCloser.to(FollowerManageIntersection)|
                    RootFollowerComeCloser.to(FollowerManageIntersection)
                    )
    
    interruption = LeaderContinuExploration.to(BehaviorInterruption)

    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier
        self.actions = self.Actions(self)

        self.behavior_set = {"name" : "DroneWaitingInStock", "number of actions" : 0}

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
        "stationary":None,
        "situation changed to root" : None,
        "situation changed" : None,
        "move done" : None,
        "centered": None,
        "aligned" : None,
        "too close" : None
        }

        self.visual_msg = None
        self.visual_indication = None

    def on_enter_state(self, state):
        if state.id != "DroneWaitingInStock":
            print(f"id: {self.identifier}, behavior: {state.id}")
            self.actions.update_behavior_set(state)
        else:
            print(f"{state.id}")
        

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
            # self.behavior_determination(dico)
            self.action_determination(dico)
            self.send(self.signature, "Module manager", "drone behavior", self.actions.drone_action)
        if title == "stationary":
            pass
        
        if title == "drone role":
            pass

        if title == "recieved visual com":
            pass

        if title == "drone role set":
            pass

        if title == "situation changed to root":
            pass
        
        if title ==  "situation changed":
            pass

        if title == "take root done":
            pass

        if title == "com send":
            pass
        
        if title == "centered":
            pass
            
        if title == "aligned":
            pass



    class Actions(StateMachine):
        Idle = State(initial=True)
        Stationary = State()
        TakeRoot = State()
        LeaveRoot = State()
        FollowTheGap = State()
        RotationToTheLeftMostGap = State()
        GetCloser = State()
        Centering = State()
        ChangeRole = State()
        ChangeSituation = State()
        Sendmsg = State()


        first_agent_start_set = (Idle.to(TakeRoot)|
                                 TakeRoot.to(ChangeRole)|
                                 ChangeRole.to(ChangeSituation)|
                                 ChangeSituation.to(Stationary)
                                 )
        
        leader_start_set = (Stationary.to(Sendmsg)|
                            Sendmsg.to(LeaveRoot)|
                            LeaveRoot.to(ChangeSituation)|
                            ChangeSituation.to(Stationary)
                            )
        
        leader_explore_branch_set = (Stationary.to(FollowTheGap)|
                                     FollowTheGap.to(Stationary)
                                     )
        
        leader_manage_intersection_set = (Stationary.to(Sendmsg)|
                                          Sendmsg.to(Centering)|
                                          Centering.to(RotationToTheLeftMostGap)|
                                          RotationToTheLeftMostGap.to(FollowTheGap)|
                                          FollowTheGap.to(Stationary)
                                          )
        
        leader_wait_set = (Stationary.to(Sendmsg)|
                           Sendmsg.to(Stationary))
        
        agent_called_set = (Idle.to(TakeRoot)|
                            TakeRoot.to(ChangeRole)|
                            ChangeRole.to(ChangeSituation)|
                            ChangeSituation.to(Stationary)
                            )
        
        root_follower_come_closer_set = (Stationary.to(Sendmsg)|
                                         Sendmsg.to(LeaveRoot)|
                                         LeaveRoot.to(ChangeSituation)|
                                         ChangeSituation.to(GetCloser)|
                                         GetCloser.to(Stationary)
                                        )
        
        follower_come_closer_set = (Stationary.to(Sendmsg)|
                                    Sendmsg.to(GetCloser)|
                                    GetCloser.to(Stationary)
                                    )
        
        follower_manage_intersection_set = (Stationary.to(Centering)|
                                            Centering.to(Stationary)
                                            )
        
        interruption_set = FollowTheGap.to(Stationary)|Stationary.to.itself(internal = True)

        def __init__(self, behavior):
            super().__init__()
            self.behavior = behavior
            self.drone_action = {"action" : None, "gap sel id" : None}

        def update_behavior_set(self, drone_behavior):

            self.behavior.behavior_set["name"] = drone_behavior.id
            if drone_behavior.id == "DroneWaitingInStock":
                self.behavior.behavior_set["number of actions"] = 0

            elif drone_behavior.id == "BehaviorInterruption":
                self.behavior.behavior_set["number of actions"] = 1

            elif drone_behavior.id == "FirstDroneStart":
                self.behavior.behavior_set["number of actions"] = 4
                # self.first_agent_start_set()
            
            elif drone_behavior.id == "LeaderLeaveTheRoot":
                self.behavior.behavior_set["number of actions"] = 4
                # self.leader_start_set()

            elif drone_behavior.id == "LeaderContinuExploration":
                self.behavior.behavior_set["number of actions"] = 2
                # self.leader_explore_branch_set()

            elif drone_behavior.id == "LeaderManageIntersection":
                self.behavior.behavior_set["number of actions"] = 5
                # self.leader_manage_intersection_set()
            
            elif drone_behavior.id == "LeaderWaiting":
                self.behavior.behavior_set["number of actions"] = 2

            elif drone_behavior.id == "CalledToEnterTheEnvironment":
                self.behavior.behavior_set["number of actions"] = 4
                # self.agent_called_set()

            elif drone_behavior.id == "FollowerComeCloser":
                self.behavior.behavior_set["number of actions"] = 3
                # self.follower_come_closer_set()

            elif drone_behavior.id == "RootFollowerComeCloser":
                self.behavior.behavior_set["number of actions"] = 5
                # self.root_follower_come_closer_set()

            elif drone_behavior.id == "FollowerManageIntersection":
                self.behavior.behavior_set["number of actions"] = 2
                # self.follower_manage_intersection_set()

        def on_enter_state(self, state):
            if state.id != "Idle":
                self.drone_action["action"] = state.id

        def on_exit_state(self, state):
            self.behavior.behavior_set["number of actions"] -= 1
            print(self.behavior.identifier, self.behavior.behavior_set, self.drone_action)

        def on_enter_ChangeRole(self):
            if self.behavior.current_state.id == "FirstDroneStart":
                self.behavior.send(self.behavior.signature, "Module manager", "set first drone role")

            if self.behavior.current_state.id == "CalledToEnterTheEnvironment":
                self.behavior.send(self.behavior.signature, "Module manager", "set drone role")
        
        def on_enter_ChangeSituation(self):
            if self.behavior.current_state.id == "FirstDroneStart":
                self.behavior.send(self.behavior.signature, "Module manager", "change situation to root")

            if self.behavior.current_state.id == "CalledToEnterTheEnvironment":
                self.behavior.send(self.behavior.signature, "Module manager", "change situation to root")

            if self.behavior.current_state.id == "LeaderLeaveTheRoot":
                self.behavior.send(self.behavior.signature, "Module manager", "change situation from root")

            if self.behavior.current_state.id == "RootFollowerComeCloser":
                self.behavior.send(self.behavior.signature, "Module manager", "change situation from root")

        def on_enter_FollowTheGap(self):
            if isinstance(self.behavior.recieved_msgs["drone situation"][1]["Intersection"], list):
                self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1
            elif self.behavior.recieved_msgs["drone situation"][1]["Corridor"]:
                self.drone_action["gap sel id"] = 1

        def on_enter_RotationToTheLeftMostGap(self):
            self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1
        
        def on_enter_Sendmsg(self):
            if self.behavior.current_state.id == "BranchReconfiguration":
                self.behavior.send(self.behavior.signature, "Module manager", "send branch reconfiguration")
            if self.behavior.current_state.id == "LeaderLeaveTheRoot" or self.behavior.current_state.id == "RootFollowerComeCloser":
                self.behavior.send(self.behavior.signature, "Module manager", "send more agent required") 
            if self.behavior.current_state.id == "LeaderManageIntersection" or self.behavior.current_state.id == "LeaderWaiting":
                self.behavior.send(self.behavior.signature, "Module manager", "send come closer")     


    def first_drone_id(self):
        return self.identifier == 0
    
    # def green_msg(self):
    #     if self.visual_msg is not None:
    #         return self.visual_msg[0][0] == "green" and self.visual_msg[0][1] == self.identifier
    #     else:
    #         print(self.identifier)
    #         return False

    def behavior_determination(self, drone_situation):

        if self.behavior_set["number of actions"] == 0:

            if drone_situation["Stock"]:
                if self.identifier == 0:
                    self.start_first_drone()
                elif self.visual_msg is not None:
                    if self.visual_msg == "green":
                        self.agent_called()
            
            elif drone_situation["Root"]:
                if self.current_state.id == "FirstDroneStart":
                    self.leader_start()
                if (self.current_state.id == "CalledToEnterTheEnvironment"
                    and self.visual_msg == "purple"):
                    self.follower_come_closer()

            elif drone_situation["Corridor"]:
                if self.current_state.id == "LeaderLeaveTheRoot":
                    self.leader_explore()

                elif self.current_state.id == "LeaderManageIntersection":
                    self.leader_explore()

                elif self.current_state.id == "RootFollowerComeCloser":
                    print("bbbb")
                    self.follower_come_closer()
                
                elif self.current_state.id == "LeaderContinuExploration" and drone_situation["Visual connectivity"][1][0][5] == "CVC dist":
                    print("yyy", drone_situation)
                    self.leader_wait_procedure()

                elif self.current_state.id == "BehaviorInterruption" and drone_situation["Visual connectivity"][1][0][4] == "NCVC obst":
                    self.leader_explore()
            
            elif isinstance(drone_situation["Intersection"], list):
                if self.current_state.id == "LeaderContinuExploration":
                    self.intersection_procedure()
                if self.current_state.id == "RootFollowerComeCloser":
                    print("cccc")
                    self.intersection_procedure()

            if self.identifier == 0 :
                print(drone_situation, self.current_state, self.recieved_msgs["drone role"][1], self.actions.drone_action)

    def action_determination(self, drone_situation):

        if (len(drone_situation["Visual connectivity"])>1
            and drone_situation["Visual connectivity"][1][0][4] == "CVC obst"
            and not drone_situation["Root"]):
            print("vvvvvvv", self.identifier)
            if self.current_state.id == "BehaviorInterruption":
                self.actions.interruption_set()
            else:
                self.interruption()
                if self.behavior_set["number of actions"] == 1:
                    self.actions.interruption_set()

        
        
        elif self.current_state.id == "FirstDroneStart":
            if self.behavior_set["number of actions"] == 4:
                self.actions.first_agent_start_set()
            if self.recieved_msgs["take root done"] is not None:
                self.actions.first_agent_start_set()
            if self.recieved_msgs["drone role set"] is not None:
                self.actions.first_agent_start_set()
            if self.recieved_msgs["situation changed to root"] is not None:
                self.actions.first_agent_start_set()
            

        elif self.current_state.id == "LeaderLeaveTheRoot":
            if self.behavior_set["number of actions"] == 4:
                self.actions.leader_start_set()
            # if self.recieved_msgs["stationary"] is not None:
            #     self.actions.leader_start_set()
            if drone_situation["Visual connectivity"][1][0][6] == "TC":
                if self.actions.current_state.id == "LeaveRoot":
                    pass
                else:
                    self.actions.leader_start_set()
            if self.visual_indication == "white":
                self.actions.leader_start_set()
            if self.recieved_msgs["situation changed"]:
                self.actions.leader_start_set()


        elif self.current_state.id == "LeaderContinuExploration":
            if self.behavior_set["number of actions"] == 2:
                self.actions.leader_explore_branch_set()
            
            if isinstance(drone_situation["Intersection"], list):
                print("wwww", drone_situation)
                self.actions.leader_explore_branch_set()
            
            if drone_situation["Visual connectivity"][1][0][5] == "CVC dist":
                print("zzzz", drone_situation["Visual connectivity"])
                self.actions.leader_explore_branch_set()
            # print("oooo", self.actions.drone_action)


        elif self.current_state.id == "LeaderManageIntersection":
            if self.behavior_set["number of actions"] == 5:
                self.actions.leader_manage_intersection_set()
            if self.recieved_msgs["com send"] is not None:
                self.actions.leader_manage_intersection_set()
            if self.recieved_msgs["centered"] is not None:
                self.actions.leader_manage_intersection_set()
            if (self.recieved_msgs["aligned"] is not None
                and drone_situation["Visual connectivity"][1][0][6] == "TC"):
                self.actions.leader_manage_intersection_set()
            if drone_situation["Corridor"]:
                self.actions.leader_manage_intersection_set()
            

        elif self.current_state.id == "CalledToEnterTheEnvironment":
            if self.behavior_set["number of actions"] == 4:
                self.actions.agent_called_set()
            if self.recieved_msgs["take root done"] is not None:
                self.actions.agent_called_set()
            if self.recieved_msgs["drone role set"] is not None:
                self.actions.agent_called_set()
            if self.recieved_msgs["situation changed to root"] is not None:
                self.actions.agent_called_set()
            

        elif self.current_state.id == "FollowerComeCloser":
            if self.behavior_set["number of actions"] == 3:
                self.actions.follower_come_closer_set()

        elif self.current_state.id == "RootFollowerComeCloser":
            if self.behavior_set["number of actions"] == 5:
                self.actions.root_follower_come_closer_set()
            if drone_situation["Visual connectivity"][1][0][6] == "TC":
                if self.actions.current_state.id == "LeaveRoot":
                    pass
                elif self.actions.current_state.id == "GetCloser":
                    pass
                else:
                    self.actions.root_follower_come_closer_set()
            if self.visual_indication == "white":
                if self.actions.current_state.id == "LeaveRoot":
 
                    self.actions.root_follower_come_closer_set()
                else:
                    pass
            if self.recieved_msgs["situation changed"]:
                self.actions.root_follower_come_closer_set()
            
            if isinstance(drone_situation["Intersection"],list):
                print("aaaa")
                self.actions.root_follower_come_closer_set()


        elif self.current_state.id == "FollowerManageIntersection":
            if self.behavior_set["number of actions"] == 2:
                self.actions.follower_manage_intersection_set()
            if self.recieved_msgs["centered"] is not None:
                self.actions.follower_manage_intersection_set()


        elif self.current_state.id == "BranchReconfiguration":
            if self.behavior_set["number of actions"] == 2:
                # self.actions.leader_explore_branch_set()
                pass
        
        self.behavior_determination(drone_situation)
        # if self.identifier ==0:
        #     print(self.actions.drone_action)
        self.recieved_msgs = dict.fromkeys(self.recieved_msgs, None)



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


            if self.current_state.id == "DroneWaitingInStock":
                for com in coms:
                    if len(com["visual msgs"])>0 and com["visual msgs"][0][1] == self.identifier:
                        self.visual_msg = com["visual msgs"][0][0]
                        print(self.identifier, self.visual_msg)

            if (self.current_state.id == "LeaderLeaveTheRoot"
                and self.actions.current_state.id == "LeaveRoot"):
                for com in coms:
                    if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
                        self.visual_indication = com["visual indications"][0]

            if (self.current_state.id == "RootFollowerComeCloser"
                and self.actions.current_state.id == "LeaveRoot"):
                for com in coms:
                    if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
                        self.visual_indication = com["visual indications"][0]
            

                        







        
    
        
    


    


