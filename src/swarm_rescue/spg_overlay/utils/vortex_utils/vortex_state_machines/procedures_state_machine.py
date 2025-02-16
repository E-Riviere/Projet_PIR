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
        "Send msg" : False,
        "Waiting for agent" : False
        }

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

        self.behavior_set = {}

        self.behavior_set_name = {
        "Drone waiting in stock" : True,
        "First drone start" : False,
        "Called to enter the environment" : False,
        "Follower come closer" : False,
        "Leader leave the root" : False,
        "Follower leave the root" : False,
        "Dead end reached" :False,
        "Being replaced at pose" : False,
        "Branch reconfiguration" : False,
        "Loop detected" : False,
        "Leader continu exploration" : False,
        "Leader manage intersection" : False,
        "Follower manage intersection" : False,
        "Root follower come closer" : False
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
                self.send(self.signature, "Module manager", "change situation from root")
            
            if self.behavior_set_name["Leader manage intersection"]:
                if self.recieved_msgs["drone situation"][1]["Visual connectivity"][1][0][5] == "TC":
                    del self.behavior_set["Stationary"]
                    self.behavior_determination()
                else:
                    pass
            
            if self.behavior_set_name["Follower manage intersection"]:
                pass
                # print("ccc")
            
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
            
        if title == "situation changed":
            if self.behavior_set_name["Leader leave the root"]:
                del self.behavior_set["Change situation"]
            if self.behavior_set_name["Root follower come closer"]:
                del self.behavior_set["Change situation"]

        if title == "com send":
            del self.behavior_set["Send msg"]
            self.behavior_determination()


        if title == "move done":
            if self.behavior_set_name["Leader continu exploration"]:
                del self.behavior_set["Follow the gap"]
            

        if title == "centered":
            # print(self.behavior_set_name)
            if self.behavior_set_name["Leader manage intersection"]:
                del self.behavior_set["Centering"]
                self.behavior_determination()
            if self.behavior_set_name["Follower manage intersection"]:
                del self.behavior_set["Centering"]
                self.behavior_determination()
        
        if title == "aligned":
            if self.behavior_set_name["Leader manage intersection"]:
                del self.behavior_set["Rotation to the left most gap"]
                self.behavior_determination()
        
        # if title == "too close":
        #     if self.behavior_set_name["Follower come closer"]:
        #         # print(self.identifier, self.recieved_msgs["drone situation"][1])
        #         del self.behavior_set["Get closer"]



    def procedure_determination(self, drone_situation):


        if len(self.behavior_set.values()) < 1:
            self.behavior_set.clear()

            if self.identifier ==1:
                print(drone_situation)

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

            elif (drone_situation["Visual connectivity"][0] is True
                  and self.recieved_msgs["drone role"][1]["Leader"]
                  and drone_situation["Corridor"]):
                
                self.behavior_set_name["Leader continu exploration"] = True
                self.clear_behavior_set_name("Leader continu exploration")
                self.behavior_set["Follow the gap"] = 0

            elif (drone_situation["Visual connectivity"][0] is True
                  and self.recieved_msgs["drone role"][1]["Leader"]
                  and isinstance(drone_situation["Intersection"], list)):
                
                # print(self.identifier, drone_situation)
                self.behavior_set_name["Leader manage intersection"] = True
                self.clear_behavior_set_name("Leader manage intersection")
                self.behavior_set["Centering"] = 0
                self.behavior_set["Rotation to the left most gap"] = 1
                self.behavior_set["Send msg"] = 2
                self.behavior_set["Stationary"] = 3
                self.behavior_set["Follow the gap"] = 4

            elif (drone_situation["Visual connectivity"][0] is True
                  and self.recieved_msgs["drone role"][1]["Follower"]
                  and isinstance(drone_situation["Intersection"], list)):
                
                # print(self.identifier, "eeeee",drone_situation)
                self.behavior_set_name["Follower manage intersection"] = True
                self.clear_behavior_set_name("Follower manage intersection")
                self.behavior_set["Centering"] = 0
                self.behavior_set["Stationary"] = 1

            elif (self.recieved_msgs["drone role"][1]["Follower"]
                  and drone_situation["Root"] is True
                  and drone_situation["Visual connectivity"][0] is True
                  and self.visual_msg == "purple"):
                
                # print(self.identifier, "gggg")
                self.behavior_set_name["Root follower come closer"]
                self.clear_behavior_set_name("Root follower come closer")
                self.behavior_set["Send msg"] = 0
                self.behavior_set["Leave root"] = 1
                self.behavior_set["Change situation"] = 3
                self.behavior_set["Get closer"] = 4

            elif (self.recieved_msgs["drone role"][1]["Follower"]
                  and drone_situation["Visual connectivity"][0] is True
                  and self.visual_msg == "purple"):
                
                # print(self.identifier, "ffff", drone_situation)
                self.behavior_set_name["Follower come closer"]
                self.clear_behavior_set_name("Follower come closer")
                self.behavior_set["Get closer"] = 0
                self.behavior_set["Stationary"] = 1
                

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

        if self.drone_behaviors["Follow the gap"]:
            if self.behavior_set_name["Leader continu exploration"]:
                # print(self.recieved_msgs["drone situation"][1]["Visual connectivity"])
                if self.recieved_msgs["drone situation"][1]["Visual connectivity"][1][0][4] == "NCVC":
                    self.drone_behaviors["Follow the gap"] = [True, 1]
                else:
                    self.drone_behaviors["Follow the gap"] = False
                    self.drone_behaviors["Stationary"] = True

            elif self.behavior_set_name["Leader manage intersection"]:
                if isinstance(self.recieved_msgs["drone situation"][1]["Intersection"], list):
                    self.drone_behaviors["Follow the gap"] = [True, self.recieved_msgs["drone situation"][1]["Intersection"][1]-1]
                if self.recieved_msgs["drone situation"][1]["Corridor"]:
                    # print("dddd")
                    del self.behavior_set["Follow the gap"]
            
        if self.drone_behaviors["Get closer"]:
            if self.behavior_set_name["Root follower come closer"]:
                if self.recieved_msgs["drone situation"][1]["Corridor"] is True:
                    pass
                elif self.recieved_msgs["drone situation"][1]["Intersection"][0] is True:
                    del self.behavior_set["Get closer"]
                    # print("aaa", self.identifier, self.recieved_msgs["drone situation"][1])
            
        if self.drone_behaviors["Rotation to the left most gap"]:
            print(self.identifier, self.recieved_msgs["drone situation"][1]["Intersection"])
            self.drone_behaviors["Rotation to the left most gap"] = [True, self.recieved_msgs["drone situation"][1]["Intersection"][1]-1]

        if self.drone_behaviors["Send msg"]:
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
            

                        



            # if self.identifier == 0:
            #     print(coms)
            #     print(self.behavior_set_name)
        

    def clear_behavior_set_name(self, set_name):
        for name in self.behavior_set_name:
            if set_name != name:
                self.behavior_set_name[name] = False
            else:
                self.behavior_set_name[name] = True






        
    
        
    


    


