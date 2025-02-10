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

        self.drone_behaviors = {
        "Stationary" : True,
        "Take root" : False,
        "Leave root" : False,
        "Follow the gap" : False,
        "Rotation to the left most gap" : False,
        "Get closer" : False,
        "Turn around" : False,
        "Centering" : False,
        "Change role" : False,
        "Change situation" : False,
        "Send msg" : False
        }

        self.recieved_requests = {
        "Need behavior" : None
        }

        self.recieved_msgs = {
        "drone situation" : None,
        "take root done" : None,
        "drone role" : None,
        "recieved visual com" : None,
        "com send" : None
        }

        self.behavior_set = {}

        self.behavior_set_name = {
        "Drone waiting in stock" : True,
        "First drone start" : False,
        "Called to enter the environment" : False,
        "Need agent closer" : False,
        "Leader leave the root" : False,
        "Follower leave the root" : False,
        "Position replacement" : False,
        "Dead end reached" :False,
        "Being replaced at pose" : False,
        "Branch reconfiguration" : False,
        "Loop detected" : False,
        "Leader continu exploration" : False
        }

        self.visual_msg = None

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
            self.send(self.signature, "Module manager", "drone behavior", self.drone_behaviors)
        
        if title == "drone role":
            pass

        if title == "recieved visual com":
            pass

        if title == "take root done":
            if self.behavior_set_name["First drone start"]:
                del self.behavior_set["Take root"]
                self.behavior_determination()
                self.send(self.signature, "Module manager", "drone behavior", self.drone_behaviors)

            if self.behavior_set_name["Called to enter the environment"]:
                del self.behavior_set["Take root"]
                self.behavior_determination()
                self.send(self.signature, "Module manager", "drone behavior", self.drone_behaviors)


        if title == "stationary":
            if self.behavior_set_name["First drone start"]:
                del self.behavior_set["Stationary"]
                self.behavior_determination()
                self.send(self.signature, "Module manager", "set first drone role", self.drone_behaviors)

            if self.behavior_set_name["Called to enter the environment"]:
                del self.behavior_set["Stationary"]
                self.behavior_determination()
                self.send(self.signature, "Module manager", "set drone role", self.drone_behaviors)

            if self.behavior_set_name["Leader leave the root"]:
                del self.behavior_set["Stationary"]
                self.send(self.signature, "Module manager",)
            
            elif self.behavior_set_name["Drone waiting in stock"]:
                del self.behavior_set["Stationary"]
                self.behavior_determination()
        
        if title == "drone role set":
            if self.behavior_set_name["First drone start"]:
                del self.behavior_set["Change role"]
                self.behavior_determination()
                self.send(self.signature, "Module manager", "change situation to root")

            if self.behavior_set_name["Called to enter the environment"]:
                del self.behavior_set["Change role"]
                self.behavior_determination()
                self.send(self.signature, "Module manager", "change situation to root")
        

        if title == "situation changed to root":
            if self.behavior_set_name["First drone start"]:
                del self.behavior_set["Change situation"]
            if self.behavior_set_name["Called to enter the environment"]:
                del self.behavior_set["Change situation"]
            

        if title == "com send":
            del self.behavior_set["Send msg"]
            self.behavior_determination()


    def procedure_determination(self, drone_situation):


        if len(self.behavior_set.values()) < 1:
            self.behavior_set.clear()

            if drone_situation["Root"] and self.recieved_msgs["drone role"][1]["Leader"]:
                self.behavior_set_name["Leader leave the root"] = True
                self.clear_behavior_set_name("Leader leave the root")
                self.behavior_set["Send msg"] = 0
                self.behavior_set["Leave root"] = 1
                self.behavior_set["Stationary"] = 2
                self.behavior_set["Change situation"] = 3
            
            # if len(self.recieved_msgs["recieved visual com"][1])>0:
            elif drone_situation["Stock"] and self.visual_msg == "green":
                self.behavior_set_name["Called to enter the environment"] = True
                self.clear_behavior_set_name("Called to enter the environment")
                self.behavior_set["Take root"] = 0
                self.behavior_set["Stationary"] = 1
                self.behavior_set["Change role"] = 2
                self.behavior_set["Change situation"] = 3

            elif drone_situation["Visual connectivity"] and self.recieved_msgs["drone role"][1]["Leader"]:
                self.behavior_set_name["Leader continu exploration"] = True
                self.clear_behavior_set_name("Leader continu exploration")

            elif drone_situation["Stock"]:
                if self.identifier == 0:
                    self.behavior_set_name["First drone start"] = True
                    self.clear_behavior_set_name("First drone start")
                    self.behavior_set["Take root"] = 0
                    self.behavior_set["Stationary"] = 1
                    self.behavior_set["Change role"] = 2
                    self.behavior_set["Change situation"] = 3

                else:
                    self.behavior_set_name["Drone waiting in stock"] = True
                    self.clear_behavior_set_name("Drone waiting in stock")
                    self.behavior_set["Stationary"] = 0
            
            

    def behavior_determination(self):
        for behavior in self.drone_behaviors:
            if behavior in self.behavior_set:
                if self.behavior_set[behavior] == min(self.behavior_set.values()):
                    self.drone_behaviors[behavior] = True
                else:
                    self.drone_behaviors[behavior] = False
            else:
                self.drone_behaviors[behavior] = False

        if self.drone_behaviors["Send msg"]:
            if self.behavior_set_name["Dead end reached"]:
                self.send(self.signature, "Module manager", "send branch reconfiguration")
            if self.behavior_set_name["Leader leave the root"]:
                self.send(self.signature, "Module manager", "send more agent required")              



    def process_communication(self, communication, vc_list):

        if len(communication) > 0:
            coms = []
            for com in communication:
                coms.append(com[1])
        
            # for com in coms:
            #     if com["id"] in vc_list:
            #         pass


            if self.behavior_set_name["Drone waiting in stock"]:
                for com in coms:
                    if len(com["visual msgs"])>0 and com["visual msgs"][0][1] == self.identifier:
                        self.visual_msg = com["visual msgs"][0][0]
                        print(self.visual_msg)

            if self.behavior_set_name["Leader leave the root"]:
                for com in coms:
                    if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
                        print(self.identifier, self.behavior_set)
                        del self.behavior_set["Leave root"]
                        self.behavior_determination()



            # if self.identifier == 0:
            #     print(coms)
            #     print(self.behavior_set_name)
        

    def clear_behavior_set_name(self, set_name):
        for name in self.behavior_set_name:
            if set_name != name:
                self.behavior_set_name[name] = False
            else:
                self.behavior_set_name[name] = True






        
    
        
    


    


